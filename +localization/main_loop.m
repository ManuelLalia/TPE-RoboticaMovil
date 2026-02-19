function [mean_pose, new_particles, state] = main_loop(map, particles, v_cmd, w_cmd, ranges, angles, Ts, state)
%MAIN_LOOP Maneja la lógica general del algoritmo de localización
%   Se llama al algoritmo del filtro de partículas, luego verifica si se
%   dan las condiciones para reducir el número de partículas y actualiza el
%   grafico con la distribución de partículas actual

% Filtro de partículas
[new_particles, mean_pose] = localization.particle_filter(map, particles, v_cmd, w_cmd, ranges, angles, Ts, state);

% Verifica el grado de convergencia y reduce las partículas 
[new_particles, state] = localization.reduce_N_particles(new_particles, state);

% Gráfico del estado actúal del filtro
figure(2); clf;
show(map); hold on;
plot(new_particles(:,1), new_particles(:,2), 'b.'); axis equal;
quiver(mean_pose(1), mean_pose(2), cos(mean_pose(3)), sin(mean_pose(3)), 2, 'Linewidth', 1.2, 'Marker', 'o');
title('Distribución inicial de partículas');

end

