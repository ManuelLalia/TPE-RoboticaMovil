function state = check_surroundings(ranges, move_count)
%CHECK_SURROUNDINGS Summary of this function goes here
%   Detailed explanation goes here

distance = 0.3;

centro = ceil(length(ranges)/2);
N = ceil(length(ranges)*0.1);
if any(ranges(centro-N:centro+N) < distance)
    state = "Cuidado";
else
    if all(ranges(1:N) < distance)
        state = "ParedDer";
    else
        if all(ranges(end-N:end) < distance)
            state = "ParedIzq";
        else
            if (move_count >= 0 && move_count <=120)
                state = "Giro360";
            else
                state = "Normal";
            end
        end
    end
end
end

