function [v_cmd, w_cmd, state, move_count] = main_movement(pose_est, state, ranges, angles, move_count, path, path_obj)
%MAIN_MOVEMENT Summary of this function goes here
%   Detailed explanation goes here

switch state
    case "Localization"
        [v_cmd, w_cmd, move_count] = movement.reactive(pose_est, ranges, angles, move_count);
        
    case "Tracking"
        [v_cmd, w_cmd, move_count] = movement.reactive(pose_est, ranges, angles, move_count);
        
    case "FollowPath"
        movement_state = movement.check_surroundings(ranges, 300); % 300 para que no entre en "Giro360"
        disp('Movement state');
        disp(movement_state);
        if movement_state == "Normal"
            [v_cmd, w_cmd, state] = movement.follow_path(pose_est, path, path_obj, state);
            disp('Entre a follow_path');
        else
            [v_cmd, w_cmd, move_count] = movement.reactive(pose_est, ranges, angles, move_count);
        end
        
    case "Stop"
        v_cmd = 0;
        w_cmd = 0;
end

