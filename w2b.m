function [H]=w2b(alpha,beta)

H=[cos(alpha)*cos(beta) -cos(alpha)*sin(beta) -sin(alpha);
    sin(beta) cos(beta) 1;
    sin(alpha)*cos(beta) -sin(alpha)*sin(beta) cos(alpha)];