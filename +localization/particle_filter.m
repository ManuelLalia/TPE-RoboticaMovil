function new_particles = particle_filter(map, particles, v_cmd, w_cmd, ranges, angles, Ts)
%PARTICLE_FILTER Summary of this function goes here
%   Detailed explanation goes here

% Muevo las particulas según el modelo de movimiento
particles = localization.sample_motion_model(particles, v_cmd, w_cmd, Ts);

distance_map = localization.create_distance_map(map);

weights = localization.measurement_model(particles, ranges, angles, map, distance_map);
disp(weights);

new_particles = localization.resample(particles, weights);

end

