function [outputArg1,outputArg2] = measurement_model(particles, ranges, angles, map, distance_map)
%MEASUREMENT_MODEL Summary of this function goes here
%   Detailed explanation goes here

N_particles = size(particles, 1);
N_rays = length(ranges);

idx_valid = ~isnan(ranges);
ranges_valid = ranges(idx_valid);
angles_valid = angles(idx_valid);

x_rays = particles(:,1) + cos(particles(:,3) + angles_valid) .* ranges_valid';
y_rays = particles(:,2) + sin(particles(:,3) + angles_valid) .* ranges_valid';

dist_matrix = inf(N_particles, N_rays);
for i=1:1
    xi = x_rays(i,:);
    yi = y_rays(i,:);
    index = world2grid(map, [xi', yi']);
    
    row = index(:, 1);
    col = index(:, 2);

    valid_rows = row >= 1 & row <= map.GridSize(1);
    valid_cols = col >= 1 & col <= map.GridSize(2);
    not_nan = ~isnan(row) & ~isnan(col);
    valid_mask = valid_rows & valid_cols & not_nan;
    
    linear_idx = sub2ind(size(distance_map), row(valid_mask), col(valid_mask));
    dist_matrix(i, valid_mask) = distance_map(linear_idx);
    
end

disp(dist_matrix)
% index_matrix = world2grid(map, [x_rays(:), y_rays(:)]);
% 
% row = index_matrix(:, 1);
% col = index_matrix(:, 2);
% 
% valid_rows = row >= 1 & row <= map.GridSize(1);
% valid_cols = col >= 1 & col <= map.GridSize(2);
% not_nan = ~isnan(row) & ~isnan(col);
% valid_mask = valid_rows & valid_cols & not_nan;
% 
% row = row(valid_mask);
% col = col(valid_mask);





end

