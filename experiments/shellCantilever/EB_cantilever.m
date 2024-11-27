% clc 
% clear all
% close all
% Y = 2e7;
Y = 2e9;
L = 0.1 - 0.01;
w = 0.02;
h = 0.001;

rho = 1200;
% rho = 7850;
mass = rho*L*w*h
mperL = rho*w*h;
g = -9.81;

I = w*h^3/12;
alpha1 = 1.875^2;

omega_n = alpha1 * sqrt(Y*I/(mperL*L^4)); % rad/s
f = omega_n/(2*pi)
Time_period = 1/f

mean_disp_tip = mass*g*L^3/(8*Y*I) % m