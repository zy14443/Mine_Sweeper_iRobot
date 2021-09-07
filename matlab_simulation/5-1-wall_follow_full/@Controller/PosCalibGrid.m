function PosCalibGrid( this )
%POSCALIBGRID: calibrate position according to the grid position
    if this.Action ~= Actions.WallFollow % Only calibrate when wall following
        this.wf_cnt = 0;    % reset w-f counter if action is not wall-following
        return
    end
    if this.wf_cnt < 5     % do not calibrate if w-f counter less than ...
        return
    end
    x = this.Robot.x;
    y = this.Robot.y;
    x_grid = this.Robot.x_grid;
    y_grid = this.Robot.y_grid;
    angle_adjust = 3/180*pi; % adjust angle theta a little bit when hit boundary limit
    switch this.Robot.Direct
        case 1 % Up
            x_limit_r = x_grid * this.Robot.gridLength - (this.Robot.radius+0.05);
            x_limit_l = x_grid * this.Robot.gridLength - (this.Robot.radius+this.Robot.rangeIR+0.05);
            if x > x_limit_r  % right hand side limit
                this.Robot.x = x_limit_r;
                this.Robot.theta = this.Robot.theta + angle_adjust;
                disp('***************position grid calib,UUUUUUUUUUrrrrrrrrrr')
            elseif x < x_limit_l % left hand side limit
                this.Robot.x = x_limit_l;
                this.Robot.theta = this.Robot.theta - angle_adjust;
                disp('***************position grid calib,UUUUUUUUUUlllllllllll')
            end
        case 2 % Down
            x_limit_r = (x_grid - 1) * this.Robot.gridLength + (this.Robot.radius-0.05);
            x_limit_l = (x_grid-1) * this.Robot.gridLength + (this.Robot.radius+this.Robot.rangeIR-0.05);
            if x < x_limit_r
                this.Robot.x = x_limit_r;
                this.Robot.theta = this.Robot.theta + angle_adjust;
                disp('***************position grid calib,DDDDDDDDDrrrrrrrrrr')
            elseif x > x_limit_l
                this.Robot.x = x_limit_l;
                this.Robot.theta = this.Robot.theta - angle_adjust;
                disp('***************position grid calib,DDDDDDDDDlllllllllll')
            end
        case 3 % Left
            y_limit_r = y_grid * this.Robot.gridLength - (this.Robot.radius+0.05);
            y_limit_l = y_grid * this.Robot.gridLength - (this.Robot.radius+this.Robot.rangeIR+0.05);
            if y > y_limit_r
                this.Robot.y = y_limit_r;
                this.Robot.theta = this.Robot.theta + angle_adjust;
                disp('***************position grid calib,LLLLLLLLLLLrrrrrrrrrr')
            elseif y < y_limit_l
                this.Robot.y = y_limit_l;
                this.Robot.theta = this.Robot.theta - angle_adjust;
                disp('***************position grid calib,LLLLLLLLLLlllllllllll')
            end
        case 4 % Right
            y_limit_r = (y_grid - 1) * this.Robot.gridLength + (this.Robot.radius-0.05);
            y_limit_l = (y_grid-1) * this.Robot.gridLength + (this.Robot.radius+this.Robot.rangeIR-0.05);
            if y < y_limit_r
                this.Robot.y = y_limit_r;
                this.Robot.theta = this.Robot.theta + angle_adjust;
                disp('***************position grid calib,RRRRRRRRrrrrrrrrrr')
            elseif y > y_limit_l
                this.Robot.y = y_limit_l;
                this.Robot.theta = this.Robot.theta - angle_adjust;
                disp('***************position grid calib,RRRRRRRRlllllllllll')
            end
    end
    
    


end

