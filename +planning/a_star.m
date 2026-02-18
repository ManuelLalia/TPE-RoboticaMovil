function path_xy = a_star(map, pose, obj)
%A_STAR Summary of this function goes here
%   Detailed explanation goes here
originalMatrix = occupancyMatrix(map);

start = [pose(1), pose(2)];
start = world2grid(map, start);

obj = world2grid(map, obj);

%##################
% occ = occupancyMatrix(map);
% dist = bwdist(occ > 0.5);
% lambda = 10;   % en celdas
% cost = exp(-dist*2 / lambda);
% map_cost = occupancyMap(cost);
% figure(6);
% show(map_cost);
% planner = plannerAStarGrid(map_cost);
%#############


inflate(map, 0.15);
occ = getOccupancy(map);
if occ(start(1), start(2)) >= 0.65
    disp('entre al if');
    disp(start);
    start = planning.find_nearest_free(map, start);
    disp(start);
end

% figure(6);
% show(map);
planner = plannerAStarGrid(map);
path = plan(planner, start, obj);

setOccupancy(map, originalMatrix);

path_xy = grid2world(map, [path(:,1), path(:,2)]);

figure(5);
show(map); hold on;
plot(path_xy(:,1), path_xy(:,2), 'r','LineWidth',2);
end

