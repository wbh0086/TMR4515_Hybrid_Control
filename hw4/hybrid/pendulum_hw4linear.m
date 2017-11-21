clear all
close all
clc

%% Basic Parameters
% parameters
M = 1;                  %[kg]    cart mass
m = 0.75;               %[kg]    pendulum mass
b = 0.1;                %[N·s/m] damper coefficient
k = 0.15;               %[N/m]   spring stiffness
g = 9.81;               %[m/s^2] gravitational acceleration
L = 1.775;              %[m]     Presumed rod length for linearized model

% matrices for systems
A = [0 0 1 0; 0 0 0 1; -k/M -m*g/M -b/M 0; k/(M*L) (m+M)*g/(M*L) b/(M*L) 0];
B = [0;0;1/M;-1/(M*L)];
E = [0 0;0 0;-1/M 0;1/(M*L) -1/(m*L)];

C = [1 0 0 0; 0 1 0 0];
D = [0 0]';

% augment model
[aw,bw,cw,dw] = tf2ss([0.64],[1 0.8]);
Aw = [aw 0;0 aw];
Bw = [1 0;0 1];
Cw = [cw 0;0 cw];

A_a = [A E*Cw; zeros(2,4) Aw];
B_a = [B; zeros(2,1)];
E_a = [zeros(4,2); Bw];
C_a = [C zeros(2,2)];

% simulation parameters
Ts = 0.001;              %[s]     sampling time
duration = 200;          %[s]     simulation time

% variables
X_0 = [0;0*pi/180;0;0;0;0];  %        initial states
u = 10;                      %[N]     Force imposed on the cart
w_p = 10;                    %        noise power of band-limited white noise w
f_p = 1e-4;                  %        noise power of band-limited white noise f
m_p = 1e-7;
%% Local controller
run localcontroller

%% Kalman filter
[Phi,Delta] = c2d(A_a,B_a,Ts);
[Phi,Gamma] = c2d(A_a,E_a,Ts);

% KF4
Q_k4 = diag([10.000101 9.9935]);
R_k4 = 1e-5*diag([1 1]);

% KF3
Q_k3 = diag([10.0000905 9.995]);
R_k3 = 1e-5*diag([1 1]);

% KF2
Q_k2 = diag([10.00007 9.997]);
R_k2 = 1e-5*diag([1 1]);

% KF1
Q_k1 = diag([10.00001 10.001]);
R_k1 = 1e-5*diag([1 1]);

%% Running Simulation
sim pdl_linear_hw4hybrid



