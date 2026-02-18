function [v_cmd, w_cmd, move_count]= reactive_mapping(pose_est, ranges, angles, move_count)
%REACTIVE Summary of this function goes here
%   Detailed explanation goes here

state = movement.check_surroundings(ranges, move_count, true);
disp(state);

switch state
    case "Cuidado"
        v_cmd = 0;
        w_cmd = 0.5;
        
    case "ParedIzq"
        v_cmd = 0.1;
        w_cmd = -0.3;
        
    case "ParedDer"
        v_cmd = 0.1;
        w_cmd = 0.3;
        
    case "Giro360"
        v_cmd = 0.1;
        w_cmd = 0.5;
        move_count = move_count + 1;
        
    case "Normal"
        if (move_count > 200 && move_count <= 240)
            v_cmd = 0.15;
            w_cmd = 0;
            move_count = move_count + 1;
        else
            if (move_count > 240 && move_count < 280)
                v_cmd = 0.15;
                w_cmd = 0;
                move_count = move_count + 1;
            else
                v_cmd = 0.1;
                w_cmd = 0;
                move_count = 201;
            end
        end
        
end
end