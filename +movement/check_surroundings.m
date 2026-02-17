% function state = check_surroundings(ranges, move_count, is_mapping)
% %CHECK_SURROUNDINGS Summary of this function goes here
% %   Detailed explanation goes here
% 
% if is_mapping
%     distance_front = 0.5;
%     distance_side = 0.8;
% else
%     distance_front = 0.3;
%     distance_side = 0.3;
% end
% 
% 
% centro = ceil(length(ranges)/2);
% percentage = 0.2;
% N = ceil(length(ranges)*percentage);
% 
% if any(ranges(centro-floor(N/2):centro+floor(N/2)) < distance_front)
%     state = "Cuidado";
% else
%     if all(ranges(1:N) < distance_side)
%         state = "ParedDer";
%     else
%         if all(ranges(end-N:end) < distance_side)
%             state = "ParedIzq";
%         else
%             if (move_count >= 0 && move_count <=120)
%                 state = "Giro360";
%             else
%                 state = "Normal";
%             end
%         end
%     end
% end
% end
% 

function state = check_surroundings(ranges, move_count, is_mapping)

if is_mapping
    distance_front = 0.5;
    distance_side  = 0.8;
else
    distance_front = 0.3;
    distance_side  = 0.3;
end

centro = ceil(length(ranges)/2);
percentage = 0.2;
N = ceil(length(ranges)*percentage);

% Frente
front_sector = ranges(centro-floor(N/2):centro+floor(N/2));
front_sector = front_sector(~isnan(front_sector));

if ~isempty(front_sector) && any(front_sector < distance_front)
    state = "Cuidado";
    return;
end

% Pared derecha
right_sector = ranges(1:N);
right_sector = right_sector(~isnan(right_sector));

if ~isempty(right_sector)
    ratio = sum(right_sector < distance_side) / length(right_sector);
    if ratio > 0.7
        state = "ParedDer";
        return;
    end
end

% Pared izquierda
left_sector = ranges(end-N:end);
left_sector = left_sector(~isnan(left_sector));

if ~isempty(left_sector)
    ratio = sum(left_sector < distance_side) / length(left_sector);
    if ratio > 0.7
        state = "ParedIzq";
        return;
    end
end

% Giro / normal
if (move_count >= 0 && move_count <= 120)
    state = "Giro360";
else
    state = "Normal";
end
end
