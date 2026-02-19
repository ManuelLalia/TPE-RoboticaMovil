function [v_cmd, w_cmd, state] = follow_path(pose, path, path_obj, state)
%FOLLOW_PATH Se utiliza un controlador para generar los comandos de
%   movimiento y seguir el camino calculado
%   Evalúa la condición de fin


controller = controllerPurePursuit;
controller.Waypoints = path;
controller.LookaheadDistance = 1;
controller.DesiredLinearVelocity = 0.15;
controller.MaxAngularVelocity = 0.5;

% Se evalúa el error entre la pose estimada y el objetivo para determinar
% si se llegó al destino
dx = path_obj(1) - pose(1);
dy = path_obj(2) - pose(2);
error = sqrt(dx^2 + dy^2);

if error < 0.2
    v_cmd = 0;
    w_cmd = 0;
    state = "Stop";
else
    [v_cmd, w_cmd] = controller(pose);
end
end

