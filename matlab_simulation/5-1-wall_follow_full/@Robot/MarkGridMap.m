function [ map_grid ] = MarkGridMap( flow_rcd, map_grid )
%MARKGRIDMAP Mark grid map according to flow record,
% mark 0 for path and 1 for obstacles
    rcd_num = size(flow_rcd,1);
    [dimy, dimx] = size(map_grid);

    for i = 1 : rcd_num
        x = flow_rcd(i,1);
        y = flow_rcd(i,2);
        if map_grid(y, x) == -1
            map_grid(y, x) = 0; % 0 meaning path
        end
    end
    % walk through again to mark the obstacles
    for i = 1 : rcd_num
        x = flow_rcd(i,1);
        y = flow_rcd(i,2);
        switch flow_rcd(i,3)
            case 1 % up, check right
                x = x + 1;
                if x <= dimx
                    if map_grid(y, x) == -1
                        map_grid(y, x) = 1;     % 1 meaning obstacle
                    end
                end
            case 2 % down, check left
                x = x - 1;
                if x >= 1
                    if map_grid(y, x) == -1
                        map_grid(y, x) = 1;     % 1 meaning obstacle
                    end
                end
            case 3 % left, check top
                y = y + 1;
                if y <= dimy
                    if map_grid(y, x) == -1
                        map_grid(y, x) = 1;     % 1 meaning obstacle
                    end
                end
            case 4 % right, check bottom
                y = y - 1;
                if y >= 1
                    if map_grid(y, x) == -1
                        map_grid(y, x) = 1;     % 1 meaning obstacle
                    end
                end
        end
    end


end

