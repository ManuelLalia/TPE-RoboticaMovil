function [distance_map, occupancy_map] = create_distance_map(map)
%CREATE_DISTANCE_MAP Summary of this function goes here
%   Detailed explanation goes here

occupancy_map = getOccupancy(map);
binary_map = occupancy_map > 0.195;

distance_map = bwdist(binary_map);
end

