clear all
%close all
clc

%% Basic Parameters
% parameters
M = 1;                  %[kg]    cart mass
m = 0.75;               %[kg]    pendulum mass
b = 0.1;                %[N·s/m] damper coefficient
k = 0.15;               %[N/m]   spring stiffness
g = 9.81;               %[m/s^2] gravitational acceleration
L = 1.4;                %[m]     Presumed rod length for linearized model

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
duration = 30;           %[s]     simulation time

% variables
X_0 = [0;0*pi/180;0;0];
u = 10;                   %[N]     Force imposed on the cart
w_p = 10;                 %        noise power of band-limited white noise w
f_p = 1e-4;               %        noise power of band-limited white noise f
m_p = 1e-7;

%% LQR
Q = diag([100 100 0 0]);
R = 10;
K = lqr(A,B,Q,R);

Cn = [1 0 0 0];
Nbar = -pinv(B)*A*pinv(Cn)+K*pinv(Cn);
%% Kalman filter
[Phi,Delta] = c2d(A,B,Ts);
[Phi,Gamma] = c2d(A,E,Ts);
Q_k = diag([1 1]);
R_k = 1e-4*diag([1 1]);

%% Running Simulation
sim pdl_nonlinear_hw3n

%% Plot
figure(1)
subplot(2,1,1)
plot(contmn.time,contmn.signals.values(:,1),est.time,est.signals.values(:,1),state_data.time,state_data.signals.values(:,1))
xlabel('time (s)')
ylabel('x (m)','position',[-3,7.5])
set(gca,'fontname','TimesNewRoman')
grid on
legend('Conted.','Est.','Non-conted')
subplot(2,1,2)
plot(contmn.time,contmn.signals.values(:,2)/pi*180,est.time,est.signals.values(:,2)/pi*180,state_data.time,state_data.signals.values(:,2)/pi*180)
xlabel('time (s)')
ylabel('\theta (deg)','position',[-3,-0])
set(gca,'fontname','TimesNewRoman')
grid on;


