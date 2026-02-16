function path_xy = a_star(map, pose, obj)
%A_STAR Summary of this function goes here
%   Detailed explanation goes here
originalMatrix = occupancyMatrix(map);

start = [pose(1), pose(2)];
start = world2grid(map, start);

obj = world2grid(map, obj);

inflate(map, 0.22)
planner = plannerAStarGrid(map);
path = plan(planner, start, obj);

setOccupancy(map, originalMatrix);

path_xy = grid2world(map, [path(:,1), path(:,2)]);

figure(5);
show(map); hold on;
plot(path_xy(:,1), path_xy(:,2), 'r','LineWidth',2);
end

