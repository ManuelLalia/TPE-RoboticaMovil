function new_particles = main_loop(map, particles, v_cmd, w_cmd, ranges, angles, Ts)
%MAIN_LOOP Summary of this function goes here
%   Detailed explanation goes here

new_particles = localization.particle_filter(map, particles, v_cmd, w_cmd, ranges, angles, Ts);

pose_var = var(new_particles, 0, 1);


figure(1); clf;
show(map); hold on;
plot(new_particles(:,1), new_particles(:,2), 'b.'); axis equal;
title('Distribución inicial de partículas');

% disp(pose_var);

% if pose_var < 0.01


end

