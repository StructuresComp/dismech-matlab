function tangential_wave = actuate_snake(n_bends, t)
    % --- Parameters you can tune ---
    amplitude = 0.02;             % Max strain applied to each bend
    frequency = 2.0;              % Oscillation frequency in Hz
    spatial_wavelength = 1.0;     % Wavelength along the body in normalized units
    phase_offset = pi / 2;        % Phase offset between vertical and tangential components

    % --- Compute strain pattern ---
    s = linspace(0, 1, n_bends);    % Normalized position along body

    % Traveling sinusoidal waves
    omega = 2 * pi * frequency;
    k = 2 * pi / spatial_wavelength;

    vertical_wave = amplitude * sin(omega * t - k * s);
    tangential_wave = amplitude * sin(omega * t - k * s + phase_offset);

end
