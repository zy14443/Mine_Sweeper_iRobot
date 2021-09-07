function CheckFWDone( this )
%CHECKFWDONE check whether wall-follower reaches its stop pose
% set the event flag of Events.WFDone
%     if this.Robot.State ~= States.WallFollow
%         return
%     end
    if size(this.Robot.grid_rcd, 1) < 2 % wait until second action to judge, thus avoid misjudge when stop pose just added
        return
    end
    if this.Robot.Mission == Missions.WFCircle
        if isequal(this.Robot.grid_rcd(1,:),this.Robot.grid_rcd(end,:))
            disp('~~~~~~~~~~~~~~WF Circle finished~~~~~~~~~~~~~~')
            Grid_rcd = this.Robot.grid_rcd
            stopPose = this.Robot.stopPoseWF
            this.Robot.WFDone = 1;
        end
    elseif this.Robot.Mission == Missions.WF2Explore
        if isequal(this.Robot.grid_rcd(end,:), this.Robot.stopPoseWF)
            disp('~~~~~~~~~~~~~~WF 2Explore finished~~~~~~~~~~~~~~')
            Grid_rcd = this.Robot.grid_rcd
            stopPose = this.Robot.stopPoseWF
            this.Robot.WFDone = 1;
        end
    elseif this.Robot.Mission == Missions.WFReturn
        if this.Robot.y < -this.Robot.radius
            disp('***WF Return Done***')
            this.Robot.WFDone = 1;
        end
%         if isequal([5, 1, 4], this.Robot.grid_rcd(end,:))
%             disp('***WF Return Done***')
%             this.Robot.WFDone = 1;
%         end
    end
    
end

