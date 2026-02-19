function [new_particles, state] = reduce_N_particles(particles, state)
%REDUCE_N_PARTICLES Reduce el número de partículas para acelerar el
%algoritmo y controla el estado de la localización
%   Evalúa la varianza del conjunto de partículas y si esta cumple ciertos
%   umbrales se reduce el total de partículas

N_mid = 300;
N_low = 100;

N_particles = size(particles, 1);

pose_var = var(particles, 0, 1);
var_xy = pose_var(1) + pose_var(2);
var_theta = pose_var(3);

disp('Calculo varianzas');
disp(var_xy);
disp(var_theta);

if var_xy<10 && var_theta<2 && N_particles > N_mid
    N_particles_new = N_mid;
    new_particles = particles(1:N_particles_new,:);
    state = "Tracking";
else
    if var_xy<4 && var_theta<0.1 && N_particles > N_low
        N_particles_new = N_low;
        new_particles = particles(1:N_particles_new,:);
        state = "FindObjetive";
    else
        new_particles = particles;
    end
end
end

