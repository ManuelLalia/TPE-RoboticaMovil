function [particles] = initialize_particles(map, N_particles)
%INITIALIZE_PARTICLES Inicializa las partículas en el mapa
particles = zeros(N_particles, 3);

x_limits = map.XWorldLimits;
y_limits = map.YWorldLimits;

count = 0;
while count < N_particles
    x = unifrnd(x_limits(1), x_limits(2));
    y = unifrnd(y_limits(1), y_limits(2));
    if getOccupancy(map, [x, y]) <= 0.1 
        count = count + 1;
        particles(count, :) = [x, y, unifrnd(-pi, pi)];
    end
end

figure(1); clf;
show(map); hold on;
plot(particles(:,1), particles(:,2), 'b.'); axis equal;
title('Distribución inicial de partículas');

end

