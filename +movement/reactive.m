function [v_cmd, w_cmd, move_count]= reactive(pose_est, ranges, angles, move_count)
%REACTIVE Summary of this function goes here
%   Detailed explanation goes here

state = movement.check_surroundings(ranges, move_count);
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
        if (move_count > 120 && move_count <= 160)
            v_cmd = 0.15;
            w_cmd = -0.2;
            move_count = move_count + 1;
        else
            if (move_count > 160 && move_count < 200)
                v_cmd = 0.15;
                w_cmd = 0.2;
                move_count = move_count + 1;
            else
                v_cmd = 0.1;
                w_cmd = 0.2;
                move_count = 120;
            end
        end
        
end
end


