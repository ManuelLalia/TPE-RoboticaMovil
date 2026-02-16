function [new_particles, state] = reduce_N_particles(particles, state)
%REDUCE_N_PARTICLES Summary of this function goes here
%   Detailed explanation goes here
N_mid = 200;
N_low = 150;

N_particles = size(particles, 1);

pose_var = var(particles, 0, 1);
var_xy = pose_var(1) + pose_var(2);
var_theta = pose_var(3);

disp('Calculo varianzas');
disp(var_xy);
disp(var_theta);

if var_xy<10 && var_theta<10 && N_particles > N_mid
    N_particles_new = N_mid;
    new_particles = particles(1:N_particles_new,:);
    state = "Tracking";
else
    if var_xy<0.05 && var_theta<0.05 && N_particles > N_low
        N_particles_new = N_low;
        new_particles = particles(1:N_particles_new,:);
        state = "FindObjetive";
    else
        new_particles = particles;
    end
end



end

