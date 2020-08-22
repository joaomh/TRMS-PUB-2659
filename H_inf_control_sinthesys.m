
%%
% Design and Implementation of Controllers on Twin Rotor System
% Department of Electrical and Computer Engineering
% University of Sao Paulo
% Copyright 2020 Joao Pinheiro
%%

%  Plant Model
load 'all_tfs'

Kp1 = 0.95;
Kp2 = 0.8;

% Yaw ok, pitch wtf
% Kp2 = 2;

% G = [tf11 tf12; tf21 tf22];
s = tf('s');
H1p = 1.359 / ( s^3 + 0.9973*s^2 + 4.7868*s + 4.278 );
H2p = 3.60 / ( s^3 + 6*s^2 + 5*s );
G = [H1p/(1 + H1p*Kp1) 0; 0 H2p/(1 + H2p*Kp2)];
G = minreal(G);
%den1 = tf22*tf11 - tf12*tf21;
%Gp1 = den1/tf22;
%Gp2 = den1/tf11;
%G = [1/(1 + Kp1*Gp1) 0; 0 1/(1 + Kp2*Gp2)];
% Weighting functions
Wp1 = tf([1 10],[1 0.001]);
Wp1 = [Wp1 0; 0 Wp1]; %% Penalize Sensitivity Function

Wu1 = tf([200 400],[1 2000]);
Wu2 = tf([400 800],[1 2000]);
Wu = [Wu1 0; 0 Wu2];  %% Penalyze Control Effort (K*S)

% Creating Augmanted Plant P
W3 = [];
P = augw(G,Wp1,Wu,W3);

% Obtaining the Controller K
[K,CL,GAM, INFO] = hinfsyn(P);

% Plotting Closed Loop TFs
L = G*K;
S = inv(eye(2)+L);
T = eye(2)-S;
%
% figure(1), clf
% sigma(S,T,{0.1, 10});
% legend('T' ,'inv(Wt)')
% figure(2), clf
% sigma(K*S, inv(Wu),{0.1,1000});
% legend('KS' ,'inv(Wu)')
% figure (3),clf
% sigma(S, inv(Wp1), {0.1,100});
% legend('S' ,'inv(Wp)')

% Isolating Controller ss matrices
a = K.a;
b = K.b;
c = K.c;
d = K.d;

% Controller order reduction
[Kr,info_add] = reduce(K,6);
% [Kr,info] = reduce(K,10,'ErrorType','mult'); % select BSTMR
%  figure(4), clf
%  bode(K,'b',Kr,'r'), grid on

a = Kr.a;
b = Kr.b;
c = Kr.c;
d = Kr.d;

% Simulink model
sim('Robust_control_final',200)