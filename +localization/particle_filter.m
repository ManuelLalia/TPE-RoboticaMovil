function [new_particles, mean_pose] = particle_filter(map, particles, v_cmd, w_cmd, ranges, angles, Ts, state)
%PARTICLE_FILTER Ejecuta los pasos del filtro de partículas:
%   Modelo de movimiento
%   Modelo de medición
%   Resample

% Muevo las particulas según el modelo de movimiento
new_particles = localization.sample_motion_model(particles, v_cmd, w_cmd, Ts, state);

% Se calculan los pesos de las patículas en base al modelo de medición
weights = localization.measurement_model_field(map, new_particles, ranges, angles);

% Se calcula la pose media ponderando las patrtículas con sus pesos
mean_pose = sum(new_particles .* weights,1);

% Se llama al algoritmo de resample
new_particles = localization.resample(new_particles, weights);

disp('mean_pose');
disp(mean_pose);

end

