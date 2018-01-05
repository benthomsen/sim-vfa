function [sys,x0,str,ts]=VFA_v1(t,x,u,flag,data,x0)
% Nonlinear dynamics of 3-wing VFA model
% Level-1 MATLAB S-function
% 
% See "Modeling for Control of Very Flexible Aircraft", Gibson et al.
% (https://doi.org/10.2514/6.2011-6202) for description

    switch flag % flag for MATLAB S-function 
        case 0
            [sys,x0,str,ts] = mdlInitializeSizes(x0);
        case 1
            sys = mdlDerivatives(t,x,u,data);
        case {2,9}
            sys = [];
        case 3
            sys = mdlOutputs(t,x,u);
        otherwise
            error(['unhandled flag = ',num2str(flag)]);
    end
end

%% mdlInitializeSizes

function [sys,x0,str,ts] = mdlInitializeSizes(x0)
% Return the sizes, initial conditions, and sample times for the S-function.

    sizes = simsizes;
    sizes.NumContStates  = 7;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 7;
    sizes.NumInputs      = 9;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    str = [];

    ts  = [0 0];   % inherited sample time

end

%% mdlDerivatives

function sys = mdlDerivatives(~,x,u,data) %(t,x,u)
% Compute derivatives for continuous states.

    V=x(1);     % velocity (ft/s)
    alpha=x(2); % angle of attack (rad)
    h=x(3);     % altitude (ft)
    theta=x(4); % pitch angle (rad)
    q=x(5);     % pitch rate (rad/s)
    eta=x(6);   % dihedral angle (rad)
    etadot=x(7);% dihedral rate (rad/s)

    thrust=u(1);  % thrust
    delta2=u(2);  % center aileron (rad)
    delta1=u(3);  % outer aileron (rad)
    deltat2=u(4); % center elevator (rad)
    deltat1=u(5); % outer elevator (rad)
    % winds:
    V2dis_X=u(6);
    V2dis_Z=u(7);
    V3dis_X=u(8);
    V3dis_Z=u(9);

    delta3=delta1;   % outer aileron
    deltat3=deltat1; % outer elevator
    gamma=theta-alpha;

    %% Density at altitude
    [~,~,rho,~]=atmosphere4(h,1);

    %% Aircraft Properties
    data_struct.w = data(1);
    data_struct.x = data(2);
    data_struct.y = data(3);
    data_struct.z = data(4);
    
    g = 32.2;
    weight1wing = data_struct.w; % weight of one wing panel (lbs)
    mstar = weight1wing/g;       % mass of one wing panel (slugs)
    m = 3*mstar;                 % total vehicle mass (slugs)
    ixxstar = mstar*data_struct.x; % (slugs ft^2)
    iyystar = mstar*data_struct.y; % (slugs ft^2)
    izzstar = mstar*data_struct.z; % (slugs ft^2)

    s = 80; % span of a single wing section (ft)
    c = 8;  % chord length of wing (ft)

    stail = 20; % span of a single tail section (ft)
    ctail = 2;  % chord length of tail (ft)

    booml = 6+30; % boom length (ft)

    rhobar = 0.5*rho*V^2; % dynamic pressure

    areastar = s*c; % area of single wing section (ft^2)

    kapd = 0.07;

    clalpha = 2*pi; % C_L_alpha
    cld = 2; % C_L_delta
    cmdelta = -0.25;
    cma = 0;
    cm0 = 0.025;
    cd0 = 0.007;

    beta = 0;

    %% Wind force calculations
    V2dis_xyz= roty(theta)*[V2dis_X 0 V2dis_Z]';
    V2dis_x=V2dis_xyz(1);
    V2dis_y=V2dis_xyz(2);
    V2dis_z=V2dis_xyz(3);

    V3dis_xyz= rotx(eta)*roty(theta)*[V3dis_X 0 V3dis_Z]';
    V3dis_x=V3dis_xyz(1);
    V3dis_y=V3dis_xyz(2);
    V3dis_z=V3dis_xyz(3);

    vcx3=V*cos(alpha)-s/6*sin(eta)*q+V3dis_x;
    vcy3=(V*sin(alpha)+etadot*s/3*cos(eta))*sin(eta)+V3dis_y;
    vcz3=(V*sin(alpha)+etadot*s/3*cos(eta))*cos(eta)-etadot*s/2+V3dis_z;

    V3=sqrt(vcx3^2+vcy3^2+vcz3^2);
    V1=V3;

    alpha3=atan2(vcz3,vcx3);
    beta3=asin(vcy3/V3);
    alpha1=alpha3;
    beta1=-beta3;

    vbx2=V*cos(alpha)+s/3*sin(eta)*q+V2dis_x;
    vby2=0;
    vbz2=V*sin(alpha)+s/3*cos(eta)*etadot+V2dis_z;

    V2=sqrt(vbx2^2+vby2^2+vbz2^2);

    alpha2=atan2(vbz2,vbx2);
    beta2=asin(vby2/V2);

    rhobar1=0.5*rho*V1^2;
    rhobar2=0.5*rho*V2^2;
    rhobar3=0.5*rho*V3^2;

    cl1=clalpha*alpha1+cld*delta1;
    cl2=clalpha*alpha2+cld*delta2;
    cl3=clalpha*alpha3+cld*delta3;

    lift1=rhobar1*cl1*areastar;
    lift2=rhobar2*cl2*areastar;
    lift3=rhobar3*cl3*areastar;

    liftt1=rhobar1*clalpha*(alpha1+deltat1)*stail*ctail;
    liftt2=rhobar2*clalpha*(alpha2+deltat2)*stail*ctail;
    liftt3=rhobar3*clalpha*(alpha3+deltat3)*stail*ctail;

    drag1=(cd0+kapd*cl1^2)*rhobar*areastar;
    drag2=(cd0+kapd*cl2^2)*rhobar*areastar;
    drag3=(cd0+kapd*cl3^2)*rhobar*areastar;

    w1=[-drag1 0 -lift1]';
    w2=[-drag2 0 -lift2]';
    w3=[-drag3 0 -lift3]';

    wt1=[0 0 -liftt1]';
    wt2=[0 0 -liftt2]';
    wt3=[0 0 -liftt3]';

    p1= rotx(eta)*w2b(alpha1,beta1)*(w1+wt1);
    p2=w2b(alpha2,beta2)*(w2+wt2);
    p3= rotx(-eta)*w2b(alpha3,beta3)*(w3+wt3);

    pt1= rotx(eta)*w2b(alpha1,beta1)*(wt1);
    pt2=w2b(alpha2,beta2)*(wt2);
    pt3= rotx(-eta)*w2b(alpha3,beta3)*(wt3);

    ptotal=p1+p2+p3;

    wtotal=inv(w2b(alpha,beta))*ptotal;

    lift=[0 0 -1]*wtotal;
    drag=[-1 0 0]*wtotal;

    %% Dynamics

    l1=s/2-s/3*sin(eta);
    l2=s/3*sin(eta);
    l3=l1;

    moment1=rhobar1*c*areastar*(cm0+cmdelta*delta1+cma*alpha1)+booml*pt1(3);
    moment2=rhobar2*c*areastar*(cm0+cmdelta*delta2+cma*alpha2)+booml*pt2(3);
    moment3=rhobar3*c*areastar*(cm0+cmdelta*delta3+cma*alpha3)+booml*pt3(3);

    moment=moment1+moment2+moment3-l1*p1(1)+l2*p2(1)-l3*p3(1);

    flapmom=-s/2*([0 0 1]*w2b(alpha3,beta3)*(w3+wt3)+mstar*g*cos(eta)*cos(theta))-70^2*eta-2*sqrt(10^10)*0.707*etadot;

    Vdot = (thrust*cos(alpha)-drag)/m-g*sin(gamma);

    alphadot=-(thrust*sin(alpha)+lift)/(m*V)+q+g*cos(gamma)/V;

    hdot=V*sin(gamma);

    thetadot=q;
    
    c1=3*iyystar;
    c2=2*izzstar-2*iyystar+mstar*s^2/6;
    qdot=(moment-2*c2*sin(eta)*cos(eta)*etadot*q)/(c1+c2*sin(eta)^2);

    d1=s/2*mstar*((Vdot*sin(alpha)+V*cos(alpha)*alphadot)*cos(eta)-V*sin(alpha)*sin(eta)*etadot-2*s/3*cos(eta)*sin(eta)*etadot^2);
    d2=(iyystar-izzstar-mstar*s^2/12)*sin(eta)*cos(eta)*q^2-s/2*mstar*cos(eta)*V*cos(alpha)*q;
    d3=ixxstar+mstar*(s^2/4+s^2/6*cos(eta^2));
    etadotdot=(flapmom+d1-d2)/d3;

    sys=[Vdot alphadot hdot thetadot qdot etadot etadotdot]';

end

%% mdlOutputs

function sys = mdlOutputs(~,x,~)%t,x,u
% Return the output vector for the S-function
    sys = [x(1); x(2); x(3); x(4); x(5); x(6); x(7)];
end
