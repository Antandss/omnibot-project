clc, clear

% syms theta theta2 theta3 theta_dot1 theta_dot2 theta_dot3 
% 
% alpha2 = deg2rad(60);
% alpha3 = deg2rad(-60);
% R = 0.100;
% r = 0.025;
% 
% G = [theta_dot1 ; theta_dot2 ; theta_dot3];
% Z = [-sin(theta) cos(theta) R ; 
%     -sin(theta+alpha2) cos(theta+alpha2) R ; 
%     -sin(theta+alpha3) cos(theta+alpha3) R];
% 
% u = [theta_dot1 ; theta_dot2 ; theta_dot3];
% A = [-R/r 0 0; 0 -R/r 0; 0 0 -R/r];
% B = [cos(theta)/r cos(theta+alpha2)/r cos(theta+alpha3)/r; 
%     -sin(theta)/r -sin(theta+alpha2)/r -sin(theta+alpha3)/r; 
%     -1/r -1/r -1/r];
% C = eye(3);
% D = zeros(3,3);
% 
% ss(A,B,C,D);
% 

%%
% System parameters
r = 0.02;
l = 0.11;
h=0.05;
% LQR parameters
Q1 = 100;
Q2 = 1;
Q12 = 1;
% Kalman parameters
G = eye(3);
R1 = eye(3);
R2 = eye(3);

Z = [ 0 1 l ; -sin((2*pi)/3) cos((2*pi)/3) l ; -sin((4*pi)/3) cos((4*pi)/3) l];
Z = (1/r) * Z;
B = r * inv(Z);

% System kinematics in ss matrices
A = [0, 0, 0;
     0, 0, 0;
     0, 0, 0];

B = [   r/(3*sqrt(3)), 0, -r/(3*sqrt(3));
        r/3, r/3, r/3 ;
        r/(3*l), -2*r/(3*l), r/(3*l)];

C = [1, 0, 0;
     0, 1, 0 ;
     0, 0, 1];

D = [0, 0, 0;
     0, 0, 0;
     0, 0, 0];

% State-space model
sysd = c2d(ss(A, B, C, D), h);
[Phi, Gam] = ssdata(sysd);

Q = Q1;
R = Q2;
N = eye(3);

[K,S, CLP] = dlqr(Phi,Gam,Q,R,N); %Calc. LQR controller
L = dlqe(Phi, G, C, R1, R2); % Calc. LQE

% Combine LQR and LQE -> LQG
[Phi_lqg, Gam_lqg, C_lqg, D_lqg] = dreg(Phi, Gam, C, D, K, L);
reg = ss(Phi_lqg, Gam_lqg, C_lqg, D_lqg, h);

Gcl = feedback(sysd, reg);
step(Gcl);
