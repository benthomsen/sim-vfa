% Simulation of (SISO) relative degree three adaptive output-feedback
% controller based on the paper "Adaptive Control for a Class of
% Multi-Input-Multi-Output Plants with Arbitrary Relative Degree",
% agumented with integral error for command tracking. The goal is to be a
% simplified introductory example to become familiar with the adaptive
% control methodology before using the MIMO SimVFA package.
% 
% Ben Thomsen
% January 2017

clear; clc;
adapt = 1; % adaptive control or baseline LQR
uncrt = 1; % uncertainty added or system completely known?
steps = 1; % step commands or sinusoidal commands

% commands for the plant output - either steps or a sine wave
if (steps)
    cmd_mag   = 1;  % command magnitude
    step_time = 15; % 15 second steps
    sine_freq = 0;
else
    % single sine wave input
    cmd_mag   = 1; % command magnitude
    sine_freq = 1; % rad/s
    step_time = 1;
end
tfin = 300;   % simulation time (s)
dT   = 0.005; % sim step size (1/Hz)

% command filter (so derivative is available)
z_cmd  = 1;   % zeta: damping ratio
om_cmd = 3.5; % omega: natural freq (rad/s)
A_cmd = [0,   1;
         -om_cmd^2, -2*om_cmd*z_cmd];
B_cmd = [0; om_cmd^2];
C_cmd = eye(2);

% nominal first-order plant parameters
a_p = 3;   % pole location (rad/s)
b_p = 1.5; % high-freq gain

% nominal second-order actuator parameters
om_n = 2.6; % omega: natural freq (rad/s)
zeta = 0.7; % zeta: damping ratio

% uncertainty - see paper for descriptions
if (uncrt)
    th_star_p  = 0.1; % \theta_{p}^{*}
    th_star_zw = 0.1; % \theta_{\zeta \omega}^{*}
    th_star_w  = 0.1; % \theta_{\omega}^{*}
    lam_star_p = 0.6; % \lambda_{p}^{*}
    lam_star_a = 0.8; % \lambda_{a}^{*}
else
    th_star_p  = 0;
    th_star_zw = 0;
    th_star_w  = 0;
    lam_star_p = 1;
    lam_star_a = 1;
end

lam_star = lam_star_p * lam_star_a; % \lambda^{*}
lam_max  = abs(lam_star);

psi_star_1 = [-th_star_p; 0; 0; 0];  % \psi_{1}^{*}
psi_star_3 = [0; -th_star_w; -th_star_zw; 0]; % \psi_{3}^{*}
psi_max    = max(norm(psi_star_1), norm(psi_star_3));

% augmented system with actuator model and integral of output error
%   first state is plant output, second and third are actuator states, 
%   fourth is integral of output error
A = [-a_p, b_p, 0, 0;
     0,    0,   1, 0;
     0, -om_n^2, -2*zeta*om_n, 0
     1,    0,   0, 0];
 
b_1 = [b_p; 0; 0; 0];    % relative degree 1 input path (for TF y(s)/u_p(s))
b_3 = [0; 0; om_n^2; 0]; % relative degree 3 input path (for TF y(s)/u(s))
c   = [1; 0; 0; 0];      % output

A_star = A + b_1*psi_star_1' + b_3*psi_star_3'; % true (unknown) dynamics in sim (A^{*})

% filter coefficients and coordinate changes - see paper for details
a20 = 1; % a_{2}^{0}
a21 = 2; % a_{2}^{1}
a22 = 1; % a_{2}^{2}

ba_3 = a22 * b_3; % b_{3}^{a}
ba_2 = a22 * A * b_3 + a21 * b_3; % b_{2}^{a}
ba_1 = a22 * A^2 * b_3 + a21 * A * b_3 + a20 * b_3; % b_{1}^{a}

% closed-loop reference model (CRM)
if (steps)
    Q = diag([1; 0.01; 0.01; 0.3]); % LQR weighting matrix
    eps = 2; % scalar gain used in calculation of feedback gain l
else
    Q = diag([1; 0.01; 0.01; 15]); % LQR weighting matrix
    eps = 4; % scalar gain used in calculation of feedback gain l
end
r = 0.1; % LQR control effort weight
k = lqr(A, b_3, Q, r)'; % LQR feedback gain
A_m = A - b_3 * k'; % reference model state matrix
l   = eps * ba_1 * b_p * om_n^2; % output-error feedback
b_r = [0; 0; 0; -1]; % "B_z" in MIMO section of paper (for integral error state)

% tuners for adaptive law
mu  = 0.2; % scalar gain in g(x, mu) = 1 + (mu * x'x)
A_h = [0,   1;
       -a20/a22, -a21/a22];
b_h = [0; a20/a22];
c_h = [1; 0];
c_h1 = [0; 1];
c_h2 = [-a20/a22; -a21/a22];

% learning rates on individual adaptive parameters
if (adapt)
    gamma = 4; % to scale all gains simultaneously
    g.a = 1 * gamma; % \psi_{3}^{1}
    g.b = 1 * gamma; % \psi_{3}^{2}
    g.c = 50 * gamma; % \lambda
    g.d = 0.5 * gamma; % \undeline{\psi}_{1}
    g.e = 0.1 * gamma; % \undeline{\psi}_{2}
    g.f = 0.5 * gamma; % \undeline{\psi}_{3}
    g.g = 50 * gamma; % \lambda^{-1} (see MIMO section of paper)
else
    g.a = 0; % \psi_{3}^{1}
    g.b = 0; % \psi_{3}^{2}
    g.c = 0; % \lambda
    g.d = 0; % \undeline{\psi}_{1}
    g.e = 0; % \undeline{\psi}_{2}
    g.f = 0; % \undeline{\psi}_{3}
    g.g = 0; % \lambda^{-1} (see MIMO section of paper)
end

% run simulation, return data in SimulationOutput object "so"
so = sim('SISO_Example_Sim.slx');

% plot output
if (adapt)
    nplots = 4;
else
    nplots = 3;
end

figure('Position',[100, 100, 1200, 250*nplots]);
subplot(nplots, 1, 1)
plot(so.tout, squeeze(so.r), 'LineWidth', 1); hold on;
plot(so.tout, squeeze(so.y), 'LineWidth', 1); 
plot(so.tout, squeeze(so.y_m), 'LineWidth', 1);
xlim([0 so.tout(end)]);
grid on;
title('Command, CRM Trajectory, Output', 'interpreter', 'latex')
set(gca,'fontsize',18)

subplot(nplots, 1, 2)
plot(so.tout, squeeze(so.u), 'LineWidth', 1);
xlim([0 so.tout(end)]);
grid on;
title('Control Signal $u$', 'interpreter', 'latex')
set(gca,'fontsize',18)

subplot(nplots, 1, 3)
plot(so.tout, squeeze(so.y) - squeeze(so.y_m), 'LineWidth', 1);
xlim([0 so.tout(end)]);
grid on;
title('Tracking Error $y - y_m$', 'interpreter', 'latex')
set(gca,'fontsize',18)

if (adapt)
    subplot(nplots, 1, 4)

    steps = length(so.tout);

    norm_psi1_ada  = squeeze(vecnorm(so.psi1T));
    norm_psi1_ada  = norm_psi1_ada/norm_psi1_ada(end);
    norm_psi2_ada  = squeeze(vecnorm(so.psi2T));
    norm_psi2_ada  = norm_psi2_ada/norm_psi2_ada(end);
    norm_psi3_ada  = squeeze(vecnorm(so.psi3T));
    norm_psi3_ada  = norm_psi3_ada/norm_psi3_ada(end);
    norm_psi31_ada = squeeze(so.psi31)/so.psi31(end);
    norm_psi32_ada = squeeze(so.psi32)/so.psi32(end);
    norm_lam_ada   = squeeze(so.lam)/so.lam(end);
    norm_ilam_ada  = squeeze(so.ilam)/so.ilam(end);

    norms = [norm_lam_ada, norm_psi1_ada, norm_psi2_ada, ...
        norm_psi31_ada, norm_psi32_ada, norm_psi3_ada, norm_ilam_ada];

    plot(so.tout, norms, 'LineWidth', 1);
    xlim([0 so.tout(end)]);
    grid on;
    title('Normalized Learned Parameters', 'interpreter', 'latex')
    h = legend('$\|\lambda\|$', '$\|\underline{\psi}_1\|$', '$\|\underline{\psi}_2\|$', '$\|\psi_3^1\|$', '$\|\psi_3^2\|$', '$\|\underline{\psi}_3\|$', '$\|\lambda\|^{-1}$');
    set(h,'Interpreter','Latex','Location','SouthEast')
    set(gca,'fontsize',18)
end