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
L = 1.025;              %[m]     Presumed rod length for linearized model

% matrices for systems
A = [0 0 1 0; 0 0 0 1; -k/M -m*g/M -b/M 0; k/(M*L) (m+M)*g/(M*L) b/(M*L) 0];
B = [0;0;1/M;-1/(M*L)];
E = [0 0;0 0;-1/M 0;1/(M*L) -1/(m*L)];

C = [1 0 0 0; 0 1 0 0];
D = [0 0]';

% augment model
[aw,bw,cw,dw] = tf2ss(0.64,[1 0.8]);
Aw = [aw 0;0 aw];
Bw = [1 0;0 1];
Cw = [cw 0;0 cw];

A_a = [A E*Cw; zeros(2,4) Aw];
B_a = [B; zeros(2,1)];
E_a = [zeros(4,2); Bw];
C_a = [C zeros(2,2)];

% simulation parameters
Ts = 0.001;              %[s]     sampling time
duration = 20;          %[s]     simulation time

% variables
X_0 = [0;0*pi/180;0;0;0;0];  %        initial states
u = 10;                      %[N]     Force imposed on the cart
w_p = 10;                    %        noise power of band-limited white noise w
f_p = 1e-4;                  %        noise power of band-limited white noise f
m_p = 1e-7;
%% Local controller
run localcontroller

%% Kalman filter
% KF1
L_1 = 1.025;

A_1 = [0 0 1 0; 0 0 0 1; -k/M -m*g/M -b/M 0; k/(M*L_1) (m+M)*g/(M*L_1) b/(M*L_1) 0];
B_1 = [0;0;1/M;-1/(M*L_1)];
E_1 = [0 0;0 0;-1/M 0;1/(M*L_1) -1/(m*L_1)];

A_a_1 = [A_1 E_1*Cw; zeros(2,4) Aw];
B_a_1 = [B_1; zeros(2,1)];
E_a_1 = [zeros(4,2); Bw];

[Phi_1,Delta_1] = c2d(A_a_1,B_a_1,Ts);
[Phi_1,Gamma_1] = c2d(A_a_1,E_a_1,Ts);

% KF2
L_2 = 1.275;

A_2 = [0 0 1 0; 0 0 0 1; -k/M -m*g/M -b/M 0; k/(M*L_2) (m+M)*g/(M*L_2) b/(M*L_2) 0];
B_2 = [0;0;1/M;-1/(M*L_2)];
E_2 = [0 0;0 0;-1/M 0;1/(M*L_2) -1/(m*L_2)];

A_a_2 = [A_2 E_2*Cw; zeros(2,4) Aw];
B_a_2 = [B_2; zeros(2,1)];
E_a_2 = [zeros(4,2); Bw];

[Phi_2,Delta_2] = c2d(A_a_2,B_a_2,Ts);
[Phi_2,Gamma_2] = c2d(A_a_2,E_a_2,Ts);

% KF3
L_3 = 1.525;

A_3 = [0 0 1 0; 0 0 0 1; -k/M -m*g/M -b/M 0; k/(M*L_3) (m+M)*g/(M*L_3) b/(M*L_3) 0];
B_3 = [0;0;1/M;-1/(M*L_3)];
E_3 = [0 0;0 0;-1/M 0;1/(M*L_3) -1/(m*L_3)];

A_a_3 = [A_3 E_3*Cw; zeros(2,4) Aw];
B_a_3 = [B_3; zeros(2,1)];
E_a_3 = [zeros(4,2); Bw];

[Phi_3,Delta_3] = c2d(A_a_3,B_a_3,Ts);
[Phi_3,Gamma_3] = c2d(A_a_3,E_a_3,Ts);

% KF4
L_4 = 1.775;

A_4 = [0 0 1 0; 0 0 0 1; -k/M -m*g/M -b/M 0; k/(M*L_4) (m+M)*g/(M*L_4) b/(M*L_4) 0];
B_4 = [0;0;1/M;-1/(M*L_4)];
E_4 = [0 0;0 0;-1/M 0;1/(M*L_4) -1/(m*L_4)];

A_a_4 = [A_4 E_4*Cw; zeros(2,4) Aw];
B_a_4 = [B_4; zeros(2,1)];
E_a_4 = [zeros(4,2); Bw];

[Phi_4,Delta_4] = c2d(A_a_4,B_a_4,Ts);
[Phi_4,Gamma_4] = c2d(A_a_4,E_a_4,Ts);
     
% KF gain
Q_k = 1e3*diag([10,1e-4]);
R_k = 1.0015e-4*diag([1 1]);

%% Running Simulation
sim pdl_linear_hw4hybrid

%% Plot
figure(1)
% subplot(2,1,1)
plot(prdata.time,prdata.signals.values(1,1:5:100001),prdata.time,prdata.signals.values(1,2:5:100002),prdata.time,prdata.signals.values(1,3:5:100003),prdata.time,prdata.signals.values(1,4:5:100004))
xlabel('time (s)')
ylabel('Probability (%)')
ylim([-0.1,1.1]);
set(gca,'fontname','TimesNewRoman')
grid on
legend('H_1','H_2','H_3','H_4')
figure(2)
subplot(2,1,1)
plot(contmn.time,contmn.signals.values(:,1),est.time,est.signals.values(:,1),state_data.time,state_data.signals.values(:,1))
xlabel('time (s)')
xlim([0,10])
ylabel('x (m)')
set(gca,'fontname','TimesNewRoman')
grid on;
legend('Cont.meas.','Est.','True')
subplot(2,1,2)
plot(contmn.time,contmn.signals.values(:,2)/pi*180,est.time,est.signals.values(:,2)/pi*180,state_data.time,state_data.signals.values(:,2)/pi*180)
xlabel('time (s)')
xlim([0,10])
ylabel('\theta (deg)')
set(gca,'fontname','TimesNewRoman')
grid on;
