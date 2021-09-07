classdef Sensors < handle
    %SENSORS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ListenerHandle
        Robot           % Robot object
    end
    
    methods
        function this = Sensors(timerobj, robotobj)
            hl = addlistener( timerobj, 'TimerEvent', @(src, evt) this.onTimerEvent );
            this.ListenerHandle = hl;
            this.Robot = robotobj;
        end
        
        function onTimerEvent(this)
%             disp('Sensors');
            [BumpRight,BumpLeft,~,~,~,BumpFront] = ...                
                               BumpsWheelDropsSensorsRoomba(this.Robot.serPort);
%             RightSonar = ReadSonar(this.Robot.serPort, 1);
%             Wall = genIR(this.Robot.serPort);
%            
%             if size(RightSonar) == 0        % Sonar No data if too close
%                 RightSonar = 0.001;
%             end
%             if Wall == 1 && RightSonar > 0.3    % Simulator has bug: will 
%                                                 % penetrate wall. 
%                                                 % Combine with Wall sensor
%                                                 % to deicde
%                 RightSonar = 0.001;
%             end
            wall_signal = ReadWallSignal(this.Robot.serPort); % use wall sensor to measure distance  
            this.Robot.dis2Wall = wall_signal;
            this.Robot.BumpRight = BumpRight;
            this.Robot.BumpFront = BumpFront;
            this.Robot.BumpLeft = BumpLeft;
            dAngle = AngleSensorRoomba(this.Robot.serPort);     % Angle increment, Unit: rad
            dDist = DistanceSensorRoomba(this.Robot.serPort);   % Distance increment, Unit: meter
            this.Robot.angleTurn = this.Robot.angleTurn + dAngle; 
            this.Robot.distRun = this.Robot.distRun + dDist;
            % Update pose = [x, y, theta] in Global Reference Frame
            this.Robot.theta = mod(this.Robot.theta + dAngle,2*pi);  %map to [0, 2*pi)
            this.Robot.x = this.Robot.x + dDist * cos(this.Robot.theta);
            this.Robot.y = this.Robot.y + dDist * sin(this.Robot.theta);
            
            if this.Robot.virtWall.ON == 1 % when virtual wall is ON, need to modify measurement
                [vwBumpFront, vwdis2Wall] = VirtWallReading(this);
%                 fprintf(1,'measurement compare: wall sensor = %d, virtWall = %d\n',wall_signal, vwdis2Wall)
                if wall_signal > 0.074 && vwdis2Wall < 0.074 % when real sensor detects no-wall, and virtual wall is detected
     %                 this.Robot.dis2Wall = min(wall_signal, vwdis2Wall);
                        this.Robot.dis2Wall = vwdis2Wall;
                        wall_signal = vwdis2Wall;
                        this.Robot.virtWall.ACT = 1;
                   
                else
                    this.Robot.virtWall.ACT = 0;
                end
%                 fprintf(1,'is virtual wall activated?: %d\n',this.Robot.virtWall.ACT )
            end
%             wall_signal
            if wall_signal > 0.074
                this.Robot.NoWall = 1;
            else
                this.Robot.NoWall = 0;
            end
            
            this.Robot.Event = bitor(bitor(bitor(bitor(bitor(BumpRight*Events.BumpRight, BumpFront*Events.BumpFront)...
                                                ,BumpLeft*Events.BumpLeft),...
                                     this.Robot.NoWall*Events.NoWall),...
                               this.Robot.TaskDone*Events.TaskDone),...
                               this.Robot.WFDone*Events.WFDone);
            
%             disp('Sensors TimerEvent!');
%             fprintf(1, 'dis2Wall: %s\n', num2str(RightSonar))
        end
        
        function [vwBumpFront, vwdis2Wall] = VirtWallReading(this)
        % Decide if the virtual wall is making sensors activated
        % check Front Bump and Wall Signal only
            vwBumpFront = 0; % default value for BumpFront: not activated
            vwdis2Wall = Inf;  % default value for dis2Wall: > sense_range
            sensAng= -1.0124; %-0.225*pi; % Sensor placement relative to front of robot
%             dis2edge = this.Robot.radius*(1 - sin(-sensAng)); % distance from wall sensor to right edge. added by Yu. 4/13
            x_sensor= this.Robot.x + this.Robot.radius*cos(this.Robot.theta+sensAng);
            y_sensor= this.Robot.y + this.Robot.radius*sin(this.Robot.theta+sensAng);
            % check wall sensor
            if (x_sensor >= this.Robot.virtWall.P1(1)) && (x_sensor <= this.Robot.virtWall.P2(1))
                vwdis2Wall = y_sensor - this.Robot.virtWall.P1(2) - 0.025; 
                if vwdis2Wall < 0.074
                    vwdis2Wall = max(0, vwdis2Wall);   % may < 0

                end
            end
%                 if vwdis2Wall > 0.074
%                 vwNoWall = 1;
%                 else
%                 vwNoWall = 0;     % wall exists
%                 end
            % check front bump
            x_front = this.Robot.x + this.Robot.radius*cos(this.Robot.theta);
            y_front = this.Robot.y + this.Robot.radius*sin(this.Robot.theta);
            if (x_front >= this.Robot.virtWall.P1(1)) && (x_front <= this.Robot.virtWall.P2(1))...
                    && this.Robot.Direct == Directs.Down && y_front <= this.Robot.virtWall.P1(2)
                vwBumpFront = 1;
            end
        end
  
    end
    
end

