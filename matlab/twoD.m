clear

theta0 = pi/2;
theta0_dot = 0;
l0 = 0.3;
l0_dot = 0.0;
T0 = 0;

g = 9.81;
m = 0.5;
bl = 0.1;   % leg damping
k0 = 1000;  % leg stiffness
e = 0.15;   % box dimension
n = 0.15;   % box dimension
I = m*(e^2 + n^2)/12; % box inertia
bw = 0.001; % wheel damping 


%% Ground
% x = transpose([theta theta_dot l l_dot])
A = [0 1 0 0;...
    g*sin(theta0)/l0    -2*l0_dot/l0    (2*l0_dot*theta0_dot + g*cos(theta0))/l0^2 - 2*T0/(m*l0^3)  -2*theta0_dot/l0;...
    0 0 0 1;...
    -g*cos(theta0)      2*l0*theta0_dot     (theta0_dot^2 - k0/m)       -bl/m];

B = transpose([0 1/(m*l0^2) 0 0]);

C = [1 0 0 0];

sys = ss(A,B,C,0);

rank(obsv(sys));
rank(ctrb(sys));

Q = [100,0,0,0;...
    0,0,0,0;...
    0,0,0,0;...
    0,0,0,0];

R = 1;

[Kg,S,P] = lqr(sys,Q,R);
Ng = rscale(sys,Kg)
Kg

%% Flight
% x = transpose([theta theta_dot])
% Af = [0 1;0 0];
Bf = transpose([0 2*(m*l0^2)^(-1)]);
Af = [0 1;0 -bw/I];
% Bf = [0;1/I];
Cf = [1 0];
Gp = ss(Af,Bf,Cf,0);
Poles = [-50+1i -50-1i];
Kf = acker(Af,Bf,Poles)
Nf = rscale(Gp,Kf)