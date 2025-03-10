%% 3 Pole System Analysis
clear;
clc;

s = tf('s');

% System parameters
g = 9.81;  
l = 0.4815; 
b = 0.00265;  
tau = 0.161;  
a = 1/tau;  

% Pendulum transfer function
G = (-s/l) / (s^2 - (g/l));

% Motor transfer function
M = (a*b) / (s+a);

% PI Controller
Kp = -1;  % Proportional gain
Ki = -3.5;  % Integral gain
K = (Kp * (s + Ki/Kp)) / s;  % PI controller
%controller_tf = 1; % P controller

L = G*M*K;

% Plot root locus
figure;
rlocus(L);
grid on;
hold on;
sgrid(0.7, sqrt(g/l));
title('Root Locus of an Inverted Pendulum');
hold off;

[k,poles] = rlocfind(L)

%% 5 Pole System Analysis
clear;
clc;

s = tf('s');

% System parameters
g = 9.81;  
l = 0.4815; 
b = 0.00265;  
tau = 0.161;  
a = 1/tau;  

% Pendulum transfer function
G = (-s/l) / (s^2 - (g/l));

% Motor transfer function
M = (a*b) / (s+a);

% Controller gains
Kp = -1;  % Proportional gain
Ki = -3.5;  % Integral gain
Jp = -1;
Ji = -1;
Ci = -1;

% Controller tfs
K = (Kp * (s + Ki/Kp)) / s;
J = Jp*(s + Ji/Jp)/s;
C = Ci/s^2;

M_nested = M*(J+C);

P_total = G*M_nested*K

figure;
rlocus(P_total);
grid on;
hold on;
sgrid(0.7, sqrt(g/l));
title('Root Locus of an Inverted Pendulum');
hold off;

%[k,poles] = rlocfind(P_total)