function new_particles = sample_motion_model(particles, v, w, Ts, state)
%SAMPLE_MOTION_MODEL Simula el movimiento de las partículas en base a los
% comandos enviados al robot. Se agrega ruido al modelo para simular el
% ruido del control

if state == "Localization"
    noise = [0.1, 0.1, deg2rad(4)];
else
    noise = [0.08, 0.08, deg2rad(1)];
end

N_particles = size(particles, 1);
new_particles = zeros(N_particles, 3);

% Se mueven las partículas según los comandos
new_particles(:,1) = particles(:,1) + v*cos(particles(:,3))*Ts;
new_particles(:,2) = particles(:,2) + v*sin(particles(:,3))*Ts;
new_particles(:,3) = particles(:,3) + w*Ts;

% Se suma el ruido Gaussiano
new_particles(:,1) = new_particles(:,1) + noise(1)*randn(N_particles,1);
new_particles(:,2) = new_particles(:,2) + noise(2)*randn(N_particles,1);
new_particles(:,3) = new_particles(:,3) + noise(3)*randn(N_particles,1);
end

