function [ ret ] = IsCovered(map_grid)
%ISCOVERED: check if the cells are all covered
% return 1 if yes
    [dimy, dimx] = size(map_grid);
    for i = 1:dimy
        for j = 1:dimx
            if map_grid(i,j) == -1
                ret = 0;
                return
            end
        end
    end
    ret = 1;
end

