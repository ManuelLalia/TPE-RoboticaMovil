function [mean_pose, new_particles, state] = main_loop(map, particles, v_cmd, w_cmd, ranges, angles, Ts, n_iter, state)
%MAIN_LOOP Summary of this function goes here
%   Detailed explanation goes here

new_particles = localization.particle_filter(map, particles, v_cmd, w_cmd, ranges, angles, Ts, n_iter, state);

% disp(n_iter);
N = size(new_particles, 1);
disp('N_particles');
disp(N);

[new_particles, state] = localization.reduce_N_particles(new_particles, state);

mean_pose = mean(new_particles,1);

figure(2); clf;
show(map); hold on;
plot(new_particles(:,1), new_particles(:,2), 'b.'); axis equal;


quiver(mean_pose(1), mean_pose(2), cos(mean_pose(3)), sin(mean_pose(3)), 2,...
    'Linewidth', 1.2, 'Marker', 'o');
title('Distribución inicial de partículas');

% disp(pose_var);

% if pose_var < 0.01


end

