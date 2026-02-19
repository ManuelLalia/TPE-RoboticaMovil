function state = check_surroundings(ranges, move_count, is_mapping)
%CHECK_SURROUNDINGS Determina la presencia de obstáculos y define el estado
%para las acciones de movimiento

% Parámetros distintos según si está localizando o mapeando
if is_mapping
    distance_front = 0.5;
    distance_side = 0.8;
    percentage = 0.2;
else
    distance_front = 0.4;
    distance_side = 0.4;
    percentage = 0.1;
end

% Medición central del lidar
centro = ceil(length(ranges)/2);
N = ceil(length(ranges)*percentage);

if any(ranges(centro-floor(N/2):centro+floor(N/2)) < distance_front)
    state = "Cuidado";
else
    if all(ranges(1:N) < distance_side)
        state = "ParedDer";
    else
        if all(ranges(end-N:end) < distance_side)
            state = "ParedIzq";
        else
            if (move_count >= 0 && move_count <=200)
                state = "Giro360";
            else
                state = "Normal";
            end
        end
    end
end

end

