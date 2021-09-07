function AnglCalibPID(this)
    % calibrate angle based on the output of PID controller
        if this.Action == Actions.WallFollow
            this.avg_angl_rcd = [this.avg_angl_rcd,mean(this.angl_rcd)];
    %                     this.avg_angl_rcd = [this.avg_angl_rcd; [xt, yt, mean(this.angl_rcd)]];
            if mean(this.angl_rcd) < 0.01 && length(this.angl_rcd) > 5 % when wall following for a while
                disp('@@@@Angle Adjusted@@@@')
                if this.Robot.Direct == 1     % When moving Up
                    this.Robot.theta = pi/2;
                elseif this.Robot.Direct == 2 % When moving Down
                    this.Robot.theta = 3/2*pi;
                elseif this.Robot.Direct == 3 % When moving Left
                    this.Robot.theta = pi;
                elseif this.Robot.Direct == 4 % When moving Right
                    this.Robot.theta = 0;
                end
            end
        else    % if not wall-following, empty angl_rcd
%                 disp('anglcalib quit')
            this.angl_rcd = []; 
            this.angl_rcd_indx = 0;
        end
%             mean_angle = mean(this.angl_rcd)
%             length_array = length(this.angl_rcd)
end