function path_xy = a_star(map, pose, obj)
%A_STAR Utiliza el algoritmo A* para encontrar la ruta desde pose hasta obj
%   Se utiliza A* con una heurística euclideana (definida por defecto)
%   Se ensanchan las paredes del mapa para evitar planificar rutas pegadas
%   a las paredes

% Se guarda el mapa para reconstruirlo luego
originalMatrix = occupancyMatrix(map);

start = [pose(1), pose(2)];
start = world2grid(map, start);

obj = world2grid(map, obj);

% Se infla el mapa
inflate(map, 0.15);

occ = getOccupancy(map);
% Si la estimación de la pose está en una celda ocupada se planifica desde
% la celda libre más cercana
if occ(start(1), start(2)) >= 0.65
    start = planning.find_nearest_free(map, start);
end

planner = plannerAStarGrid(map);
path = plan(planner, start, obj);

% Se reconstruye el mapa (inflate lo pisa)
setOccupancy(map, originalMatrix);

path_xy = grid2world(map, [path(:,1), path(:,2)]);

% Se grafica el camino encontrado
figure(5);
show(map); hold on;
plot(path_xy(:,1), path_xy(:,2), 'r','LineWidth',2);
end

