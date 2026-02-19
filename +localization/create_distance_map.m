function [distance_map, occupancy_map] = create_distance_map(map)
%CREATE_DISTANCE_MAP Crea el mapa que contiene la distancia de cada píxel
%al obstáculo más cercano

occupancy_map = getOccupancy(map);
binary_map = occupancy_map > 0.195;

distance_map = bwdist(binary_map);
end

