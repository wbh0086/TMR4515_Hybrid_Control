
Ts = 0.001;       %[s] sampling time

% Loading controllers
load K_A1;
load K_A2;
load K_A3;
load K_A4;

% Build state-space models
LC1_C = ss(A_kA1,B_kA1,C_kA1,D_kA1);
LC2_C = ss(A_kA2,B_kA2,C_kA2,D_kA2);
LC3_C = ss(A_kA3,B_kA3,C_kA3,D_kA3);
LC4_C = ss(A_kA4,B_kA4,C_kA4,D_kA4);

% Discretize Local controllers  
LC1_D = c2d(LC1_C,Ts);
LC2_D = c2d(LC2_C,Ts);
LC3_D = c2d(LC3_C,Ts);
LC4_D = c2d(LC4_C,Ts);

% Obtain the transfer matrix of the controller
% G1 = LC1_D.C*(eye(15)-LC1_D.A)^(-1)*LC1_D.B+LC1_D.D;
% G2 = LC2_D.C*(eye(28)-LC2_D.A)^(-1)*LC2_D.B+LC2_D.D;
% G3 = LC3_D.C*(eye(34)-LC3_D.A)^(-1)*LC3_D.B+LC3_D.D;
% G4 = LC4_D.C*(eye(14)-LC4_D.A)^(-1)*LC4_D.B+LC4_D.D;