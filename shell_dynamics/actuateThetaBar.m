function hinge_springs = actuateThetaBar(hinge_springs, hinge_spring_ind_to_actuate,curr_time)
%% CAUTION: this is a custom actuation for flapping motion in stingray, 
% if some other kind of actuation is needed, one can create a similar function

% flap_freq = 4;
flap_freq = 3.384;
% flap_amplitude = pi/2; % works
flap_amplitude = 2*pi/3; % works
n = size(hinge_spring_ind_to_actuate,1);

% % % no phase difference
% % theta = flap_amplitude/2*(cos(2*flap_freq*pi*curr_time) - 1);
% % thetas = theta.*ones(n);

%% thetas phase difference
% total_phase_difference = 3*pi; % between front end and back end % odd*pi % works
total_phase_difference = pi; % between front end and back end % odd*pi % works

del_phase = total_phase_difference/(n-1);
thetas = zeros(n,1);
for i=1:n
    phase = (i-1)*del_phase;
    thetas(i) = flap_amplitude/2*(cos(2*flap_freq*pi*curr_time + phase) - 1);
end

%% change the theta_bar in hinge springs
n_hinges_to_actuate = n;
for c=1:n_hinges_to_actuate
    hinge_springs(hinge_spring_ind_to_actuate(c)).thetaBar = thetas(c);
end
end