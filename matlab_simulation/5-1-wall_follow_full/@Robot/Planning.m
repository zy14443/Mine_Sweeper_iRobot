function [ stop, target, exploreDirect ] = Planning( map_grid, flow_map )
%PLANNING decide which grid to stop at, and which is the targeted unkown
% grid, find the pair with minimum cost
% input: 1. grid map, 2. flow map
% output: grid coordination of two grids, exploreDirect: direction from
% stop cell to target cell
    weightFwd = 1; % less cost for forward exploreDirect
    weightDist = 0;
%     grid_dist = 2;
    [dimy, dimx] = size(map_grid);
    cost = Inf; %
for grid_dist = 1:2    % find at most for two-grid distance
    for y = 1 : dimy
        for x = 1 : dimx
             if map_grid(y, x) == -1 % find a cell unexplored
                
                % check top cell
                if y + grid_dist <= dimy
                    if flow_map(y+grid_dist,x).Left > 0 && flow_map(y+grid_dist,x).Left + grid_dist * weightDist < cost
                        cost = flow_map(y+grid_dist,x).Left + grid_dist * weightDist;
                        stop = [x, y+grid_dist, Directs.Left];
                        target = [x, y];
                        exploreDirect = Directs.Left;
                    end
                    if flow_map(y+grid_dist,x).Down > 0 && flow_map(y+grid_dist,x).Down * weightFwd + grid_dist * weightDist < cost
                        cost = flow_map(y+grid_dist,x).Down*weightFwd + grid_dist * weightDist;
                        stop = [x, y+grid_dist, Directs.Down];
                        target = [x, y];
                        exploreDirect = Directs.Forward;
                    end
                end
                % check bottom cell
                if y - grid_dist >= 1 
                    if flow_map(y-grid_dist,x).Right > 0 && flow_map(y-grid_dist,x).Right + grid_dist * weightDist < cost
                        cost = flow_map(y-grid_dist,x).Right + grid_dist * weightDist;
                        stop = [x, y-grid_dist, Directs.Right];
                        target = [x, y];
                        exploreDirect = Directs.Left;
                    end
                    if flow_map(y-grid_dist,x).Up > 0 && flow_map(y-grid_dist,x).Up * weightFwd + grid_dist * weightDist < cost
                        cost = flow_map(y-grid_dist,x).Up * weightFwd + grid_dist * weightDist;
                        stop = [x, y-grid_dist, Directs.Up];
                        target = [x, y];
                        exploreDirect = Directs.Forward;
                    end
                end
                % check right cell
                if x + grid_dist <= dimx 
                    if flow_map(y,x+grid_dist).Up > 0 && flow_map(y,x+grid_dist).Up + grid_dist * weightDist < cost
                        cost = flow_map(y,x+grid_dist).Up + grid_dist * weightDist;
                        stop = [x+grid_dist, y, Directs.Up];
                        target = [x, y];
                        exploreDirect = Directs.Left;
                    end
                    if flow_map(y,x+grid_dist).Left > 0 && flow_map(y,x+grid_dist).Left * weightFwd + grid_dist * weightDist < cost
                        cost = flow_map(y,x+grid_dist).Left * weightFwd + grid_dist * weightDist;
                        stop = [x+grid_dist, y, Directs.Left];
                        target = [x, y];
                        exploreDirect = Directs.Forward;
                    end
                end
                % check left cell
                if x - grid_dist >= 1
                    if flow_map(y,x-grid_dist).Down > 0 && flow_map(y,x-grid_dist).Down + grid_dist * weightDist < cost
                        cost = flow_map(y,x-grid_dist).Down + grid_dist * weightDist;
                        stop = [x-grid_dist, y, Directs.Down];
                        target = [x, y];
                        exploreDirect = Directs.Left;
                    end
                    if flow_map(y,x-grid_dist).Right > 0 && flow_map(y,x-grid_dist).Right * weightFwd + grid_dist * weightDist < cost
                        cost = flow_map(y,x-grid_dist).Right * weightFwd + grid_dist * weightDist;
                        stop = [x-grid_dist, y, Directs.Right];
                        target = [x, y];
                        exploreDirect = Directs.Forward;
                    end
                end

            end % outer if
        end
    end
if cost ~= Inf % return if one distance cell found
    break; 
end
end %for grid_dist = 1:2    

end

