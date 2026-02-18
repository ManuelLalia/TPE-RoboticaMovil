function weights = measurement_model_field(map, particles, ranges, angles)
%MEASUREMENT_MODEL_FIELD Summary of this function goes here
%   Detailed explanation goes here
sigma = 1.1; %0.9
[distance_map, occupancy_map] = localization.create_distance_map(map);

N_particles = size(particles, 1);
N_rays = length(ranges);

% Solo evalúo los rayos que no son NaN
idx_valid_ranges = ~isnan(ranges);
valid_ranges = ranges(idx_valid_ranges);
valid_angles = angles(idx_valid_ranges);

% Proyecto los rayos de cada partícula
x_rays = particles(:, 1) + cos(particles(:, 3) + valid_angles) .* valid_ranges';
y_rays = particles(:, 2) + sin(particles(:, 3) + valid_angles) .* valid_ranges';

% Si no hay rayos válidos asigno pesos uniformes
if all(isnan(ranges))
    weights = ones(N_particles,1) / N_particles;
    return
end

% Paso de coordenadas xy a indices de matriz
grid_coords = world2grid(map, [x_rays(:), y_rays(:)]);
grid_rows = grid_coords(:,1);
grid_cols = grid_coords(:,2);

% Verifico que todos los rayos estén dentro del mapa
row_mask = grid_rows >= 1 & grid_rows <= size(distance_map, 1);
col_mask = grid_cols >= 1 & grid_cols <= size(distance_map, 2);
nan_mask = ~isnan(grid_rows) & ~isnan(grid_cols);
grid_mask = row_mask & col_mask & nan_mask; 

% Se filtran las posiciones válidas
grid_rows = grid_rows(grid_mask);
grid_cols = grid_cols(grid_mask);

% En vez de indexar por (fila, columna) indexo linealmente para mayor eficiencia
linear_index = sub2ind(size(distance_map), grid_rows, grid_cols);

% Matriz que almacena la distancia de cada rayo al obstáculo más cercano
% Se inicializa en inf
dist_matrix = inf(N_particles, N_rays);

% Asigno la distancia correspondiente a los rayos que caen en zonas válidas
dist_matrix(grid_mask) = distance_map(linear_index);

% Identifico las zonas grises del mapa (0 -> gris, 1 -> blanco o negro)
non_gray_area = (occupancy_map < 0.19) | (occupancy_map > 0.7);

% Superpongo la máscara de rayos válidos con la de zonas grises
% Finalmente tengo solo los rayos válidos que caen en zonas conocidas del mapa
final_mask = false(N_particles, N_rays);
final_mask(grid_mask) = non_gray_area(linear_index);

% Todo lo que no cumpla con ambas condiciones se ignora
dist_matrix(~final_mask) = NaN;

% % Ahora calculo los pesos según la distancia de los rayos a un obstáculo
% log_w = -0.5 * mean((dist_matrix/sigma).^2, 2, 'omitnan');
% 
% % log_w = log_w - max(log_w);
% weights = exp(log_w);
% weights = weights / sum(weights);

likelihoods = 0.99*exp(-0.5 * (dist_matrix / sigma).^2) + 0.01;
likelihoods(isnan(likelihoods)) = 0; 

% Se normalizan los pesos para que sumen 1
weights_vector = sum(likelihoods, 2);
% disp('weights');
% disp([min(weights_vector) max(weights_vector)]);
weights = weights_vector / sum(weights_vector); 

end


