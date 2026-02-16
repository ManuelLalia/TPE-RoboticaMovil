function [new_particles, mean_pose] = particle_filter(map, particles, v_cmd, w_cmd, ranges, angles, Ts, n_iter, state)
%PARTICLE_FILTER Summary of this function goes here
%   Detailed explanation goes here

% Muevo las particulas según el modelo de movimiento
new_particles = localization.sample_motion_model(particles, v_cmd, w_cmd, Ts, state);

% distance_map = localization.create_distance_map(map);

if mod(n_iter,2)==0
    weights = localization.measurement_model(new_particles, ranges, angles, map, state);
    disp('max weight');
    disp(max(weights));
    mean_pose = sum(new_particles .* weights,1);
    new_particles = localization.resample(new_particles, weights);
else
    mean_pose = mean(new_particles,1);
end
disp('mean_pose');
disp(mean_pose);

end

