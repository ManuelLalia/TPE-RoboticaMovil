function new_particles = resample(particles, weights)
%RESAMPLE Summary of this function goes here
%   Detailed explanation goes here

N_particles = size(particles,1);

% Calcular numero efectivo
N_eff = 1 / sum(weights.^2);

threshold = 0.5 * N_particles;

if N_eff < threshold
    new_particles = zeros(size(particles));
    
    c = cumsum(weights);
    u = rand() / N_particles;

    i = 1;
    for j = 1:N_particles
        while u > c(i)
            i = i + 1;
        end
        new_particles(j,:) = particles(i,:);
        u = u + 1/N_particles;
    end

else
    % No resamplear
    new_particles = particles;
end

% new_particles = zeros(size(particles));
% 
% N_particles = size(particles, 1);
% c = cumsum(weights);
% 
% u = rand() / N_particles;
% 
% i = 1;
% for j = 1:N_particles
%     while(u > c(i))
%         i = i+1;
%     end
%     new_particles(j,:) = particles(i,:);
%     u = u + 1/N_particles;
% end

end

