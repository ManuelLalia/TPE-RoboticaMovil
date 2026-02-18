function new_particles = sample_motion_model(particles, v, w, Ts, state)
%SAMPLE_MOTION_MODEL Summary of this function goes here
%   Detailed explanation goes here
if state == "Localization"
    k = 0.25;
    noise = [k, k, deg2rad(10)];
else
    k = 0.1;
    noise = [k, k, deg2rad(1)];
end

N_particles = size(particles, 1);
new_particles = zeros(N_particles, 3);

new_particles(:,1) = particles(:,1) + v*cos(particles(:,3))*Ts;
new_particles(:,2) = particles(:,2) + v*sin(particles(:,3))*Ts;
new_particles(:,3) = particles(:,3) + w*Ts;

% Ruido Gaussiano
new_particles(:,1) = new_particles(:,1) + noise(1)*randn(N_particles,1);
new_particles(:,2) = new_particles(:,2) + noise(2)*randn(N_particles,1);
new_particles(:,3) = new_particles(:,3) + noise(3)*randn(N_particles,1);
end

