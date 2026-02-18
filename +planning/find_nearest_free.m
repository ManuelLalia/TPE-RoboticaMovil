function [p_free] = find_nearest_free(map, p)
%FIND_NEAREST_FREE Summary of this function goes here
%   Detailed explanation goes here
show(map);
res = map.Resolution;
radius = ceil(0.5 * res);   % 0.5 m de búsqueda inicial

% grid = world2grid(map, p);
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
%                 p_free = grid2world(map, [x y]);
                p_free = [x y];
                return;
            end
        end
    end
end

% fallback
p_free = p;
end

