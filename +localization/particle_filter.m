function new_particles = particle_filter(map, particles, v_cmd, w_cmd, ranges, angles, Ts, n_iter, state)
%PARTICLE_FILTER Summary of this function goes here
%   Detailed explanation goes here

% Muevo las particulas según el modelo de movimiento
new_particles = localization.sample_motion_model(particles, v_cmd, w_cmd, Ts, state);

% distance_map = localization.create_distance_map(map);

if mod(n_iter,2)==0
    weights = localization.measurement_model(new_particles, ranges, angles, map, state);
%     disp(weights);
    new_particles = localization.resample(new_particles, weights);
end

end

