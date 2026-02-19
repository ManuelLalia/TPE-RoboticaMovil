function [v_cmd, w_cmd, state, move_count] = main_movement(pose_est, state, ranges, move_count, path, path_obj)
%MAIN_MOVEMENT Genera los comandos de movimiento en función del estado del algoritmo

switch state
    case "Localization"
        [v_cmd, w_cmd, move_count] = movement.reactive(ranges, move_count);
        
    case "Tracking"
        [v_cmd, w_cmd, move_count] = movement.reactive(ranges, move_count);
        
    case "FollowPath"
        % Se verifica si hay obstáculos cerca o se puede seguir el camino
        % 300 para que no entre en "Giro360" y false xq no esta mapeando
        movement_state = movement.check_surroundings(ranges, 300, false);
        
        if movement_state == "Normal"
            [v_cmd, w_cmd, state] = movement.follow_path(pose_est, path, path_obj, state);
            disp('Siguiendo el camino');
        else
            [v_cmd, w_cmd, move_count] = movement.reactive(ranges, move_count);
        end
        
    case "Stop"
        v_cmd = 0;
        w_cmd = 0;
end

