clear all
close all
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
X_0 = [0;1*pi/180;0;0];

% matrices for systems
A = [0 0 1 0; 0 0 0 1; -k/M -m*g/M -b/M 0; k/(M*L) (m+M)*g/(M*L) b/(M*L) 0];
B = [0;0;1/M;-1/(M*L)];
E = [0 0;0 0;-1/M 0;1/(M*L) -1/(m*L)];

% transfer function of the open-loop system
s = tf('s');
C = [1 0 0 0; 0 1 0 0];
D = [0 0]';
G = C*(s*eye(4)-A)^(-1)*B+D;
[zeros,poles,gains] = zpkdata(G);

% simulation parameters
Ts = 0.001;              %[s]     sampling time
duration = 10;          %[s]     simulation time

% variables
u = 0.175;               %[N]     Force imposed on the cart
w_p = 0;                 %        noise power of band-limited white noise w
f_p = 0;                 %        noise power of band-limited white noise f

%% Running Simulation
% choose linear model or nonlinear model
k_model = 1;            % input 0 to run linear model; input 1 to run nonlinear model

if k_model == 0
    sim pdl_linear_hw1
elseif k_model == 1
    sim pdl_nonlinear_hw1
end

%% Plot
figure(1)
subplot(4,1,1)
plot(state_data.time,state_data.signals.values(:,1))
xlabel('time (s)')
ylabel('x (m)','position',[-6,0])
set(gca,'fontname','TimesNewRoman')
subplot(4,1,2)
plot(state_data.time,state_data.signals.values(:,2)/pi*180)
xlabel('time (s)')
ylabel('\theta (deg)','position',[-6,-200])
set(gca,'fontname','TimesNewRoman')
subplot(4,1,3)
plot(linear_acceleration.time,linear_acceleration.signals.values(:,1))
xlabel('time (s)')
ylabel('accel. (m/s^2)','position',[-6,0])
set(gca,'fontname','TimesNewRoman')
subplot(4,1,4)
plot(linear_velocity.time,linear_velocity.signals.values(:,1))
xlabel('time (s)')
ylabel('vel. (m/s)','position',[-6,0])
set(gca,'fontname','TimesNewRoman')



