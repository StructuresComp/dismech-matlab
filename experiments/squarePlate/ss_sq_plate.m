% simply supported square plate

Y = 200e9;
mu = 0.3;
rho = 7750; 
a = 0.1;
h = 0.001;
D = Y*h^3/(12*(1-mu^2));

g = -9.81;
mass = rho*a^2*h;
Weight = mass*g;

disp_center = 0.00406*Weight*a^2/D % m
