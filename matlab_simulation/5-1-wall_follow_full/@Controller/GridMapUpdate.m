function GridMapUpdate( this )
%GRIDMAPUPDATE: update grid map at every control step, according to current pose
    tol = 0.04; % tolerance for robot on grid
    hit_length = this.Robot.gridLength/2 - tol; %- this.Robot.radius + tol;
    hit_length2 = tol; % 4cm x 4cm: hit area for direction judgement
    x = this.Robot.x;
    y = this.Robot.y;
    x_grid = ceil(x / this.Robot.gridLength); % possible grid
    y_grid = ceil(y / this.Robot.gridLength);
    x_grid = ValueProject(x_grid, 1, 6);      % Projcet value to [1,6]
    y_grid = ValueProject(y_grid, 1, 6);
    x_center = (x_grid - 0.5) * this.Robot.gridLength; % center point
    y_center = (y_grid - 0.5) * this.Robot.gridLength;
%     if this.Action == Actions.WallFollow
%         direct = this.Robot.Direct;
%     else % not wall-following, i.e. turning
%         direct = 0;     % then direction not defined
%     end
    if abs(x - x_center) < hit_length && abs(y - y_center) < hit_length
    % compute distance from center point
        this.Robot.x_grid = x_grid; % update grid position
        this.Robot.y_grid = y_grid;
        direct = this.Robot.Direct;
        if this.Action == Actions.WallFollow &&... % record start from wall-following
                (abs(x - x_center) < hit_length2 && (direct == 3 || direct == 4)) ||...
                (abs(y - y_center) < hit_length2 && (direct == 1 || direct == 2))
                        
            direct = this.Robot.Direct;
        else
            direct = 0;
        end
        % put grid position into record
        if size(this.Robot.grid_rcd,1) == 0     % if array is empty
            this.Robot.grid_rcd = [x_grid, y_grid, direct];
        else
            if this.Robot.grid_rcd(end,1) ~= x_grid ||...
                        this.Robot.grid_rcd(end,2) ~= y_grid   % if in different grid
                this.Robot.grid_rcd = [this.Robot.grid_rcd;
                                      [x_grid, y_grid, direct]];
            elseif this.Robot.grid_rcd(end,3) ~= direct        % if same grid but different action
                if this.Robot.grid_rcd(end,3) == 0
                    this.Robot.grid_rcd(end,3) = direct;
                elseif direct ~= 0  % non-0 can replace 0, 0 cannot add after non-0
                    this.Robot.grid_rcd = [this.Robot.grid_rcd;
                                      [x_grid, y_grid, direct]];
                end
            end
        end
        if this.Robot.map_grid(y_grid, x_grid) == -1 % update map
            this.Robot.map_grid(y_grid, x_grid) = this.Robot.grid_cnt;
            this.Robot.grid_cnt = this.Robot.grid_cnt+1;
        end
    end
end
function [output] = ValueProject( value, min, max )
    if value < min
        output = min;
    elseif value > max
        output = max;
    else
        output = value;
    end
end
