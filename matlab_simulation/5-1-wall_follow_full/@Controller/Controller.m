classdef Controller < handle
    properties
        ListenerHandle
        Robot              % Robot object
        serPort            % Serial Port connecting to irobot
        Mode = 1           % Control Mode
        
        dis2Wall = 0.025% + 0.025     % distance to wall to be kept during wall following
%         angle = 0
        %PD parameters
        parP
        parD
        
        CTE                % Cross Track Error
        preCTE = 0         % Previous Cross Track Error
        angl_rcd
        angl_rcd_indx = 0  % index of the angle_record array
        avg_angl_rcd       % for debug, average angle record
        wf_cnt = 0         % wall-following counter, for PosCalibGrid(), only calibrate after continuous w-f action
        % for turning
        preAE = 0              % previous angle error
        % for distance control
        preDE = 0
        
        Action          % Action
    end
    methods
        function this = Controller( timerobj, robotobj )
            hl = addlistener( timerobj, 'TimerEvent', @(src, evt) this.onTimerEvent );
            this.ListenerHandle = hl;
            this.Robot = robotobj;
            this.Action = robotobj.Action;
%             this.serPort = serPorts;
%             addlistener( robotobj, 'BumpFront', 'PostSet', @this.BumpResponse );
            addlistener( robotobj, 'Action', 'PostSet', @this.ActionResponse );
        end
        
        function onTimerEvent(this)
%             disp('Controller');
%             mode = this.Robot.Mode
%             pre = this.Robot.preMode
%             front = this.Robot.BumpFront
%             right = this.Robot.BumpRight
%             nowall = this.Robot.NoWall
%             dis2Wall = this.Robot.dis2Wall
            fprintf(1, 'pose [x, y, theta] = [%f, %f, %f], direction = %d, State = %d\n',...
                        this.Robot.x, this.Robot.y, this.Robot.theta, this.Robot.Direct, this.Robot.State)
            this.Robot.x_rcd = [this.Robot.x_rcd, this.Robot.x]; % record the trajectory
            this.Robot.y_rcd = [this.Robot.y_rcd, this.Robot.y];
            % Ture pose
            [xt yt ~]= genOverhead(this.Robot.serPort);
            this.Robot.Pose_true_rcd = [this.Robot.Pose_true_rcd;
                                        [xt yt]];
            
%             [x, y, theta] = deal(this.Robot.x, this.Robot.y,
%             this.Robot.theta)
           
            
            switch this.Action%this.Robot.Mode  %mode
                case Actions.Stop
                    StopCreate(this)
                    disp('------------stop')
                    
                case Actions.WallFollow          % Wall Follower
                    WallFollow(this, this.Robot.vel)
%                     SetFwdVelAngVelCreate(this.Robot.serPort, rand*.5, rand*.2)
                    disp('-----------wall follow')
%                     this.avg_angl_rcd = [this.avg_angl_rcd,mean(this.angl_rcd)];
% %                     this.avg_angl_rcd = [this.avg_angl_rcd; [xt, yt, mean(this.angl_rcd)]];
%                     if mean(this.angl_rcd) < 0.002
%                         disp('==00000000000000000000000000=')
%                     end

                case 2          % 2: Turn Left
                    TurnLeftCreate(this, 0.2/2)
                    disp('---------------left')
                    
                case 3          % 3: Turn Rigt
                    TurnRightCreate(this, 0.2)
                    disp('----------------right')
                case Actions.Task 
                    RunTasks(this)
                    disp('--------------RunTasks')
                case Actions.DodgeLeft
                    this.DodgeLeft(this.Robot.vel/2)
                    disp('---------Dodge to left')
                case 7
                    disp('--------------RunDist')
                    if RunDist(this, 0.1, 1, 0.001) == 1
                        disp('done')
                        this.Robot.TaskInfo.num = this.Robot.TaskInfo.num - 1;
                        tasknum = this.Robot.TaskInfo.num
                    end
                    
                case 6
                    TurnRadius(this)
                    disp('--------------TurnPID')
            end
            %if this.Robot.virtWall.ACT == 0 % calibrate only when virtual wall is not activated
                this.PosCalibGrid();
                % Calibrate angle based on PID controller output
                this.AnglCalibPID();
            %end
            this.GridMapUpdate();   % Update grid map
            this.CheckFWDone();     % check if wall following is done
        end
        
        function StopCreate(this)
            SetDriveWheelsCreate(this.Robot.serPort, 0, 0)
        end
        
        function WallFollow(this, vel)
        % Wall Following algorithm
        % Using PID controller
            cte = this.Robot.dis2Wall - this.dis2Wall;
            pre_cte = this.preCTE;
            angl = Controller.PID(cte, pre_cte);
            this.preCTE = cte;
            SetFwdVelAngVelCreate(this.Robot.serPort, vel, angl)
            % record angl 10 history
            this.angl_rcd(this.angl_rcd_indx + 1) = abs(angl);
            this.angl_rcd_indx = mod(this.angl_rcd_indx + 1, 10);
            this.wf_cnt = this.wf_cnt + 1; % increase w-f counter
        end
        function TurnLeftCreate(this, vel)
            SetDriveWheelsCreate(this.Robot.serPort, vel, - vel)
        end
        function TurnRightCreate(this, vel)
            SetDriveWheelsCreate(this.Robot.serPort, - vel, vel)
        end
        function DodgeLeft(this, vel)
            SetDriveWheelsCreate(this.Robot.serPort, vel, - vel)
        end
        function ActionResponse(this,src,evnt)
            disp('Action changed')
            this.Action = evnt.AffectedObject.Action;
        end
        
        function BumpResponse(this,src,evnt)
        % Stop the robot
        % serPort is the serial port number (for controlling the actual robot).
            switch src.Name
                case 'BumpFront'
                    disp('BumpFront')
                    if evnt.AffectedObject.BumpFront == 1
                        SetDriveWheelsCreate(this.Robot.serPort, 0,0)
                        disp('stop:Controller')
                        this.Mode = 0;
                    end
            end
        end
        function RunTasks(this)
        % Execute tasks in TaskSet
            if this.Robot.TaskInfo.num == 0
                SetDriveWheelsCreate(this.Robot.serPort, 0,0)
                this.Robot.TaskDone = 1;
                disp('task Done')
                return
            end
            indx = this.Robot.TaskInfo.current; % current task index
            task = this.Robot.TaskSet(indx);
            switch task.type
                case 1                       % travel a specifiied distance
                    disp('-------travel distance')
                    if RunDist(this, task.vel, task.dist2Run,task.tol) == 1
                        disp('done')
                        this.Robot.TaskInfo.num = this.Robot.TaskInfo.num - 1;
                        this.Robot.TaskInfo.current = this.Robot.TaskInfo.current + 1;
                        % need to clear variables used
                        this.Robot.distRun = 0;
                        this.preDE = 0;
                    end
                    
                case 2
                    disp('-------turn angle')
                    if TurnRadius(this, task.vel, task.radiusTurn, task.angle2Turn, task.tol) == 1
                        disp('done')
                        this.Robot.TaskInfo.num = this.Robot.TaskInfo.num - 1;
                        this.Robot.TaskInfo.current = this.Robot.TaskInfo.current + 1;
                        % need to clear variables used
                        this.Robot.angleTurn = 0;
                        this.preAE = 0;
                    end
            end
        end
        
        function [done] = TurnRadius(this, velTurn, radiusTurn, angle2Turn, tol)
        % Turn specified radian
        % velTurn: scale value of velocity
        % radiusTurn: specified radius of the turn, sign indicates
        % curve direction
        % angle2Turn: radian to turn, sign indicates direction on curve
            agl = this.Robot.angleTurn;
            if radiusTurn == 0
                sign = 1;
            else
%                 sign = radiusTurn/abs(radiusTurn);
                sign = (radiusTurn > 0) - (radiusTurn < 0);
            end
            ae = angle2Turn - this.Robot.angleTurn * sign;
            if abs(ae) < tol
                done = 1;
                return
            end
            pre_ae = this.preAE;
            vel = Controller.PIDturn(ae, pre_ae, velTurn, radiusTurn);
            this.preAE = ae;
            if radiusTurn == 0
                SetDriveWheelsCreate(this.Robot.serPort, vel, -vel);
            else
                SetFwdVelRadiusRoomba(this.Robot.serPort, vel, radiusTurn);    
            end
            
            done = 0; % not finished yet
        end
        
        function [done] = RunDist(this, velDist, dist2Run, tol)
        % move specified distance
        % velDist: absolute value of the velocity
        % dist2Run: specified distance, negative value means moving
        % backwards
        % tol: tolerance
        % return done = 1 when finished
            dist = this.Robot.distRun;
            de = dist2Run - this.Robot.distRun;
            if abs(de) < tol
                done = 1; % task finished
                return
            end
            pre_de = this.preDE;
            vel = Controller.PIDdist(de, pre_de, velDist);
            this.preDE = de;
            SetDriveWheelsCreate(this.Robot.serPort, vel, vel);
            done = 0; % not finished yet
        end
        
        PosCalibGrid(this)
        AnglCalibPID(this)      % calibrate angle based on the output of PID controller
        GridMapUpdate(this)     % update grid map
        

    end
    methods(Static)
        function [du] = PID(cte, pre_cte)
        %-------------------------------------------------
        % PID controller for wall follower
        % input: cte error
        % output: du, angular control
        %-------------------------------------------------
            max_du = pi / 6.0;
            para_P = 0.4;
            para_D = 20;%20;
            diff_cte = cte - pre_cte;
            du = -para_P * cte - para_D * diff_cte;
            if du > max_du
                du = max_du;
            end
            if du < -max_du
                du = -max_du;
            end
        end
        function [dv] = PIDturn(ae, pre_ae, vel, radius)
        %----------------------------------
        % PID controller for turning radius
        % input: angle error, max velocity at turning
        % output: dv, velocity control
        %----------------------------------
            para_P = 4;
            para_D = 2;
%             angVel = vel/abs(radius);
            diff_ae = ae - pre_ae;
            
%             if ae > angVel/4
%                 dv = vel;
%             else
%                 dv = ae * para_P * vel + diff_ae * para_D * vel;
%             end
            dw = ae * para_P + diff_ae * para_D;
            if radius == 0 
                dv = dw * 0.1;   % about the radius of the robot
            else
                dv = dw * abs(radius);
            end
            if abs(dv) > vel
                dv = vel * (dv/abs(dv));
            end
            
        end
        function [dv] = PIDdist(de, pre_de, vel)
        %----------------------------------
        % PID controller for moving specific distance
        % input: distance error, max velocity at moving
        % output: dv, velocity control
        %----------------------------------
            para_P = 4;
            para_D = 2;
            diff_de = de - pre_de;
%             if de > vel/4
%                 dv = vel;
%             else
%                 dv = de * para_P * vel + diff_de * para_D * vel;
%             end
            dv = de * para_P + diff_de * para_D * vel;
            if abs(dv) > abs(vel)
%                 dv = vel * (dv/abs(dv)); % same sign as dv
                dv = vel * ((dv > 0)-(dv < 0)); % same sign as dv
            end
        end
       
    end
end



