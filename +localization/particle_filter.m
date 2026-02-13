function particle_filter(map, particles, vel, ranges, angles, Ts)
%PARTICLE_FILTER Summary of this function goes here
%   Detailed explanation goes here

% Muevo las particulas según el modelo de movimiento
particles = particles + vel * Ts;

distance_map = localization.create_distance_map(map);

weights = localization.measurement_model(particles, ranges, angles, map, distance_map);



end

