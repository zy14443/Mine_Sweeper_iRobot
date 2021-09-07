function [ new_mission ] = Plan( this, mission )
% PLANNING Plan next cycle of wall following mission
% input: current mission
% use: this.grid_rcd, this.map_grid
% change: this.stopPoseWF, this.exploreDirect
% return: new mission
switch mission % mission to plan
    case Missions.WF2Explore
        if Robot.IsCovered(this.map_grid)
            disp('***mission turned to WFReturn***')
            new_mission = Missions.WFReturn;
        else
            new_mission = Missions.WFCircle;
        end
        
    case Missions.WFCircle
        Outer_Flow = this.grid_rcd;
        Outer_Flow(end,:) = []; % delete last entry, the same as the first one
        if isempty(this.outer_flow_rev)
            this.outer_flow_rev =  flipud(Outer_Flow);
        end
%         Outer_Flow_Map(1:this.gridSize,1:this.gridSize) = struct('Up',-1,'Down',-1,'Left',-1,'Right',-1); % initial value -1
        Outer_Flow_Map = Robot.Flow2Map(Outer_Flow, this.gridSize, this.gridSize); 
        this.map_grid = Robot.MarkGridMap(Outer_Flow, this.map_grid);
        % check if the whole map is coved after MarkGridMap (possibly mark an obstacle)
        if Robot.IsCovered(this.map_grid) % if all covered, plan to return
            [this.stopPoseWF, ~, this.exploreDirect ] = Robot.PlanningReturn( this.outer_flow_rev, Outer_Flow_Map );
            this.virtWall.ON = 0;      % turn off virtual wall
            disp('***return planed***')
        else
            [this.stopPoseWF, ~, this.exploreDirect ] = Robot.Planning( this.map_grid, Outer_Flow_Map );
        end
        Robot.PrintFlowMap(Outer_Flow_Map)
        Robot.GridMapPrint(this.map_grid)
        new_mission = Missions.WF2Explore;
end

end

