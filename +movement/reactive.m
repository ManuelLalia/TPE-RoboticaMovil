function [v_cmd, w_cmd, move_count]= reactive(ranges, move_count)
%REACTIVE Movimiento reactivo para evitar obstáculos

state = movement.check_surroundings(ranges, move_count, false);
disp(state);

switch state
    case "Cuidado"
        v_cmd = 0;
        w_cmd = 0.5;
        
    case "ParedIzq"
        v_cmd = 0.08;
        w_cmd = -0.4;
        
    case "ParedDer"
        v_cmd = 0.08;
        w_cmd = 0.4;
        
    case "Giro360"
        v_cmd = 0.1;
        w_cmd = 0.3;
        move_count = move_count + 1;
        
    case "Normal"
        if (move_count > 200 && move_count <= 150)
            v_cmd = 0.15;
            w_cmd = -0.1;
            move_count = move_count + 1;
        else
            if (move_count > 150 && move_count < 200)
                v_cmd = 0.15;
                w_cmd = 0.1;
                move_count = move_count + 1;
            else
                v_cmd = 0.1;
                w_cmd = 0.0;
                move_count = 201;
            end
        end
        
end
end


