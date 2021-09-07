function [ stop, target, exploreDirect ] = PlanningReturn( outer_flow_rev, flow_map )
% PlanningReturn decide which cell to stop at, and which is the targeted
% cell on the outer flow, find the pair with minimum cost. only consider
% the same cell case, otherwise may exist wall in between
% input: 1. outer flow reverse, 2. flow map of current circle
% output: grid coordination of two grids, exploreDirect: direction from
% stop cell to target cell
%     weightFwd = 1; % less cost for forward exploreDirect
%     weightDist = 0;
%     grid_dist = 2;
    len = size(outer_flow_rev,1); % length of outer flow reverse
%     [dimy, dimx] = size(flow_map);
    cost = Inf; %
    for out_dist = 1 : len  % distance to the exit on the outer flow 
        x = outer_flow_rev(out_dist,1);
        y = outer_flow_rev(out_dist,2);
        direct = outer_flow_rev(out_dist,3);
        if direct > 0 % turning cell not being used
            rev_dir = DirRev(direct); % direction on current flow should be reverse
            switch rev_dir
                case Directs.Up % find minimum cost cell
                    if flow_map(y,x).Up > 0 && flow_map(y,x).Up + out_dist < cost
                        cost = flow_map(y,x).Up + out_dist;
                        stop = [x, y, Directs.Up];
                        target = [x, y];
                        exploreDirect = Directs.Left;
                    end
                case Directs.Down
                    if flow_map(y,x).Down > 0 && flow_map(y,x).Down + out_dist < cost
                        cost = flow_map(y,x).Down + out_dist;
                        stop = [x, y, Directs.Down];
                        target = [x, y];
                        exploreDirect = Directs.Left;
                    end
                case Directs.Left
                    if flow_map(y,x).Left > 0 && flow_map(y,x).Left + out_dist < cost
                        cost = flow_map(y,x).Left + out_dist;
                        stop = [x, y, Directs.Left];
                        target = [x, y];
                        exploreDirect = Directs.Left;
                    end                
                case Directs.Right
                    if flow_map(y,x).Right > 0 && flow_map(y,x).Right + out_dist < cost
                        cost = flow_map(y,x).Right + out_dist;
                        stop = [x, y, Directs.Right];
                        target = [x, y];
                        exploreDirect = Directs.Left;
                    end                   
            end
        end
    end
%     % --- Planning org ---
% for grid_dist = 1:2    % find at most for two-grid distance
%     for y = 1 : dimy
%         for x = 1 : dimx
%              if map_grid(y, x) == -1 % find a cell unexplored
%                 
%                 % check top cell
%                 if y + grid_dist <= dimy
%                     if flow_map(y+grid_dist,x).Left > 0 && flow_map(y+grid_dist,x).Left + grid_dist * weightDist < cost
%                         cost = flow_map(y+grid_dist,x).Left + grid_dist * weightDist;
%                         stop = [x, y+grid_dist, Directs.Left];
%                         target = [x, y];
%                         exploreDirect = Directs.Left;
%                     end
%                     if flow_map(y+grid_dist,x).Down > 0 && flow_map(y+grid_dist,x).Down * weightFwd + grid_dist * weightDist < cost
%                         cost = flow_map(y+grid_dist,x).Down*weightFwd + grid_dist * weightDist;
%                         stop = [x, y+grid_dist, Directs.Down];
%                         target = [x, y];
%                         exploreDirect = Directs.Forward;
%                     end
%                 end
%                 % check bottom cell
%                 if y - grid_dist >= 1 
%                     if flow_map(y-grid_dist,x).Right > 0 && flow_map(y-grid_dist,x).Right + grid_dist * weightDist < cost
%                         cost = flow_map(y-grid_dist,x).Right + grid_dist * weightDist;
%                         stop = [x, y-grid_dist, Directs.Right];
%                         target = [x, y];
%                         exploreDirect = Directs.Left;
%                     end
%                     if flow_map(y-grid_dist,x).Up > 0 && flow_map(y-grid_dist,x).Up * weightFwd + grid_dist * weightDist < cost
%                         cost = flow_map(y-grid_dist,x).Up * weightFwd + grid_dist * weightDist;
%                         stop = [x, y-grid_dist, Directs.Up];
%                         target = [x, y];
%                         exploreDirect = Directs.Forward;
%                     end
%                 end
%                 % check right cell
%                 if x + grid_dist <= dimx 
%                     if flow_map(y,x+grid_dist).Up > 0 && flow_map(y,x+grid_dist).Up + grid_dist * weightDist < cost
%                         cost = flow_map(y,x+grid_dist).Up + grid_dist * weightDist;
%                         stop = [x+grid_dist, y, Directs.Up];
%                         target = [x, y];
%                         exploreDirect = Directs.Left;
%                     end
%                     if flow_map(y,x+grid_dist).Left > 0 && flow_map(y,x+grid_dist).Left * weightFwd + grid_dist * weightDist < cost
%                         cost = flow_map(y,x+grid_dist).Left * weightFwd + grid_dist * weightDist;
%                         stop = [x+grid_dist, y, Directs.Left];
%                         target = [x, y];
%                         exploreDirect = Directs.Forward;
%                     end
%                 end
%                 % check left cell
%                 if x - grid_dist >= 1
%                     if flow_map(y,x-grid_dist).Down > 0 && flow_map(y,x-grid_dist).Down + grid_dist * weightDist < cost
%                         cost = flow_map(y,x-grid_dist).Down + grid_dist * weightDist;
%                         stop = [x-grid_dist, y, Directs.Down];
%                         target = [x, y];
%                         exploreDirect = Directs.Left;
%                     end
%                     if flow_map(y,x-grid_dist).Right > 0 && flow_map(y,x-grid_dist).Right * weightFwd + grid_dist * weightDist < cost
%                         cost = flow_map(y,x-grid_dist).Right * weightFwd + grid_dist * weightDist;
%                         stop = [x-grid_dist, y, Directs.Right];
%                         target = [x, y];
%                         exploreDirect = Directs.Forward;
%                     end
%                 end
% 
%             end % outer if
%         end
%     end
% if cost ~= Inf % return if one distance cell found
%     break; 
% end
% end %for grid_dist = 1:2    

end
function [rev_dir] = DirRev(direct)
% find the reverse direction of given one
    switch direct
        case Directs.Up
            rev_dir = Directs.Down;
        case Directs.Down
            rev_dir = Directs.Up;
        case Directs.Left
            rev_dir = Directs.Right;
        case Directs.Right
            rev_dir = Directs.Left;
    end
end
