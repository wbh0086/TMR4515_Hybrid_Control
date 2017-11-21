clear all
%close all
clc

%% Basic Parameters
% parameters
M = 1;                  %[kg]    cart mass
m = 0.75;               %[kg]    pendulum mass
b = 0.1;                %[N·s/m] damper coefficient
k = 0.15;               %[N/m]   spring stiffness
sat_linear = 5*pi/180;  %[rad]   linear saturation for theta in linear model
sat_nl = pi/2;          %[raf]   saturation for theta in nonlinear model
g = 9.81;               %[m/s^2] gravitational acceleration
L = 1.4;                %[m]     Presumed rod length for linearized model
X_0 = [0;0*pi/180;0;0];

% matrices for systems
A = [0 0 1 0; 0 0 0 1; -k/M -m*g/M -b/M 0; k/(M*L) (m+M)*g/(M*L) b/(M*L) 0];
B = [0;0;1/M;-1/(M*L)];
E = [0 0;0 0;-1/M 0;1/(M*L) -1/(m*L)];

% transfer function of the open-loop system
s = tf('s');
C = [1 0 0 0; 0 1 0 0];
D = [0 0]';

% simulation parameters
Ts = 0.001;              %[s]     sampling time
duration = 50;          %[s]     simulation time

% variables
u = 10;               %[N]     Force imposed on the cart
w_p = 10;                 %        noise power of band-limited white noise w
f_p = 1e-4;                 %        noise power of band-limited white noise f
m_p = 1e-7;

% % augment model
% [aw,bw,cw,dw] = tf2ss([0.64],[1 0.8]);
% Aw = [aw 0;0 aw];
% Bw = [1 0;0 1];
% Cw = [cw 0;0 cw];
% 
% A_a = [A E*Cw; zeros(2,4) Aw];
% B_a = [B; zeros(2,1)];
% E_a = [zeros(4,2); Bw];
% C_a = [C zeros(2,2)];

%% LQR
Q = diag([100 100 0 0]);
R = 10;
K = lqr(A,B,Q,R);

Cn = [1 0 0 0];
Nbar = -pinv(B)*A*pinv(Cn)+K*pinv(Cn);

% Nbar = -3.1658;   % Q = diag([1000 100 0 0 0 0]); R = 100;
% K = [-3.3158,-60.3004,-5.3273,-19.8027,-0.7168,3.4665];

%% Kalman filter
[Phi,Delta] = c2d(A,B,Ts);
[Phi,Gamma] = c2d(A,E,Ts);
Q_k = diag([1 1]);
R_k = 1e-4*diag([1 1]);

%% Running Simulation
% choose linear model or nonlinear model
k_model = 1;            % input 0 to run linear model; input 1 to run nonlinear model

if k_model == 0
    sim pdl_linear_hw3n
elseif k_model == 1
    sim pdl_nonlinear_hw3n
end

%% Plot
% figure(1)
% subplot(4,1,1)
% plot(state_data.time,state_data.signals.values(:,1))
% xlabel('time (s)')
% ylabel('x (m)','position',[-6,0])
% set(gca,'fontname','TimesNewRoman')
% subplot(4,1,2)
% plot(state_data.time,state_data.signals.values(:,2)/pi*180)
% xlabel('time (s)')
% ylabel('\theta (deg)','position',[-6,-200])
% set(gca,'fontname','TimesNewRoman')
% subplot(4,1,3)
% plot(linear_acceleration.time,linear_acceleration.signals.values(:,1))
% xlabel('time (s)')
% ylabel('accel. (m/s^2)','position',[-6,0])
% set(gca,'fontname','TimesNewRoman')
% subplot(4,1,4)
% plot(linear_velocity.time,linear_velocity.signals.values(:,1))
% xlabel('time (s)')
% ylabel('vel. (m/s)','position',[-6,0])
% set(gca,'fontname','TimesNewRoman')



