function [p_free] = find_nearest_free(map, p)
%FIND_NEAREST_FREE Se encuentra la celda libre más cercana dentro de un
%radio de 50cm

res = map.Resolution;
radius = ceil(0.5 * res);   % 0.5 m de búsqueda inicial

occ  = occupancyMatrix(map);
for r = 1:radius
    for dx = -r:r
        for dy = -r:r
            x = p(1)+dx;
            y = p(2)+dy;

            if x < 1 || y < 1 || x > size(occ,2) || y > size(occ,1)
                continue;
            end

            if occ(x,y) < 0.18
                p_free = [x y];
                return;
            end
        end
    end
end
p_free = p;
end

