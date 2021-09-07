classdef Robot < handle
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        serPort
       
        x
        y
        theta =  0 %pi/2 % initial theta
        
        virtWall    % virtual wall structure: struct('ON','P1','P2')
        virtWallGrid = [5,1]
        
        x_rcd = []
        y_rcd = []
        
        Pose_true_rcd = []
                
        map_grid = -1*ones(6,6) % grid map
        % location in grid, initial location
        x_grid = 5%5%5
        y_grid = 1%2%1
        grid_cnt = 1 % count the sequence of grid being visited
        grid_rcd
        outer_flow_rev = []  % store the reverse order flow of the outmost one, used in PlanningReturn() to return to exit
        
        TaskInfo           % Number of tasks to be executed, and current task being executed
        TaskSet            % Set of tasks to be executed
        % variables for wall follower
        vel = 0.2          % vel = (velL + velR) / 2
        dis2Wall
        stopPoseWF %= [5, 3, 4]         % stop pose for wall follower
        exploreDirect = Directs.Forward                  % direction to explore after WF is done: forward:1, left:3
        Mission =  Missions.WFCircle%Missions.WFReturn%       % Wall following mission type: WFCircle, WF2Explore
        
        % variables for turning
        velTurn = 0.05
        radiusTurn = (0.025+0.1651)
        angleTurn = 0   % angle turned
        angle2Turn = pi/2  % target angle
        
        % variables for distance control
        velDist = 0.1
        dist2Run = 1    % distance to run
        distRun = 0     % distance moved
        
        preMode = 0            % Previous Mode
        
    end
    properties (SetObservable, AbortSet)
        Event
        State = States.Stop%States.WallFollow
        Action = Actions.Stop%Actions.WallFollow
        BumpFront = 0
        BumpLeft = 0
        BumpRight = 0
        TaskDone = 0
        NoWall = 0
        WFDone = 0
    end
    properties (Dependent = true)
        Mode           % Robot mode: 0: Wait; 1: Wall Follow; 2: Turn Left; 3: Turn Right; 4: U-Turn
        Direct         % Direction of the robot: 1:Up, 2:Down, 3:Left, 4:Right
    end
    properties(Constant)
        radius = 0.1651
        axleLength = 0.258
        minVel = -0.5
        maxVel = 0.5
        minAngVel = -2.5
        maxAngVel = 2.5
        minRadius = -2
        maxRadius = 2
        gridLength = 0.61
        gridSize = 6
        rangeIR = 0.1 % wall sensor range
    end
    
    methods
        function this = Robot(serPort)
            this.serPort = serPort;
            this.TaskInfo = struct('num',1,'current',1);
            for i=1:10  % set max number of tasks in TaskSet to be 10
                this.TaskSet = [this.TaskSet, struct('type',[],'vel',[],'radiusTurn',[],'angle2Turn',[],'dist2Run',[],'target',[],'tol',[])];
            end
%             this.TaskSet(1) = struct('type',1,'vel',0.2,'radiusTurn',[],'angle2Turn',[],'dist2Run',1,'target',[],'tol',0.001);
%             this.TaskSet(1) = struct('type',2,'vel',0.1,'radiusTurn',-0.2,'angle2Turn',-pi/2,'dist2Run',[],'target',[],'tol',0.001);
%             this.TaskSet(3) = struct('type',1,'vel',0.2,'radiusTurn',[],'angle2Turn',[],'dist2Run',1,'target',[],'tol',0.001);
%             this.TaskSet(4) = struct('type',2,'vel',0.1,'radiusTurn',-0.2,'angle2Turn',pi/2,'dist2Run',[],'target',[],'tol',0.001);

            addlistener( this, 'Event', 'PostSet', @this.StateEventResponse );
            addlistener( this, 'State', 'PostSet', @this.StateEventResponse );
            % Location of the robot
%             this.x = (this.x_grid - 0.5) * this.gridLength;
%             this.y = (this.y_grid - 0.5) * this.gridLength;
            % initial position use true position
            [xt yt ~]= genOverhead(this.serPort)
            this.x = xt+1.83;
            this.y = yt+1.83;
            this.virtWall = struct('ON',0,'ACT',0,...
                'P1',[(this.virtWallGrid(1) - 1.5) *  this.gridLength, (this.virtWallGrid(2) - 1) * this.gridLength],...
                'P2',[this.virtWallGrid(1) *  this.gridLength, (this.virtWallGrid(2) - 1) * this.gridLength]);
%             this.virtWall.ON = 1; % turn on virtual wall
%             this.virtWall.P1 = [(this.x_grid - 1) *  this.gridLength, this.y_grid * this.gridLength]; % end of virtual wall 
%             this.virtWall.P2 = [this.x_grid *  this.gridLength, this.y_grid * this.gridLength];       % end of virtual wall
            
            this.AddTasks(States.Enter)
            this.virtWall.ON = 1;      % turn on virtual wall
            this.State = States.Enter; % set initial state
%             this.State = States.WallFollow;
        end
        function StateEventResponse(this,src,evnt)
%             disp('event occurs')
            fprintf(1, 'event occurs: %s\n', src.Name)
            state = this.State
            event = this.Event
            switch this.State
                case States.WallFollow
                    if this.NoWall == 1
%                         this.Action = Actions.Stop;
%                         this.State = States.Stop;
%                         this.TaskInfo.num = 2;
%                         this.TaskInfo.current = 1;
%                         this.TaskSet(1) = struct('type',1,'vel',0.2,'radiusTurn',[],'angle2Turn',[],'dist2Run',0.1,'target',[],'tol',0.001);
%                         this.TaskSet(2) = struct('type',2,'vel',0.1,'radiusTurn', -0.2,'angle2Turn', pi/2,'dist2Run',[],'target',[],'tol',0.001);
%                         this.distRun = 0;           % Clear odometer for new task
%                         this.angleTurn = 0;
                        this.AddTasks(States.TurnRight);
                        this.State = States.TurnRight;
                    elseif this.BumpFront == 1
                        this.PosCalibBF()     % Calibrate Position when front bumped
                        this.AddTasks(States.TurnLeft90);
                        this.State = States.TurnLeft90;
                    elseif this.BumpRight == 1
                        this.Action = Actions.DodgeLeft;
                    else
                        this.Action = Actions.WallFollow;
                        if this.WFDone
                            if this.Mission == Missions.WFCircle
                                disp('~~~~~~~~~~~~~~WF Circle finished~~~~~~~~~~~~~~')
                                this.Mission = this.Plan(Missions.WFCircle); % go to explore, keep in wall following
%                                 this.State = States.Stop;
                                this.WFDone = 0;
                                mission = this.Mission
                                stopPose = this.stopPoseWF
                            elseif this.Mission == Missions.WF2Explore
                                disp('~~~~~~~~~~~~~~WF2Explore finished~~~~~~~~~~~~~~')
                                this.Mission = this.Plan(Missions.WF2Explore); % wall following will go to circle again
                                this.WFDone = 0;
                                mission = this.Mission
                                if this.exploreDirect == Directs.Left % exploreDirect is given by the above plan()
                                    this.AddTasks(States.TurnLeft2Explore);
                                    this.State = States.TurnLeft2Explore;
                                elseif this.exploreDirect == Directs.Forward
%                                  this.AddTasks(States.Forward2Explore);
%                                  this.State = States.Forward2Explore;
                                    this.AddTasks(States.Explore);
                                    this.State = States.Explore;
                                end
                            elseif this.Mission == Missions.WFReturn
                                disp('~~~~~~~~~~~~~~All covered~~~~~~~~~~~~~~')
                                this.State = States.Stop;
                                this.Action = Actions.Stop;
                            end
                        end
                    end
%                     %!!!!!!!!!!for test only!!!!!!!
%                     if this.Mission == Missions.WFReturn
%                         this.State = States.Stop;
%                     end
%                     %!!!!!!!!! to be deleted 4/29/12
                case States.Stop
                    this.Action = Actions.Stop;
                    disp('####in Stop State####')
                case States.TurnRight
%                     this.Action = Actions.Task;   
                    if this.BumpRight == 1
                        disp('Right Bumped when turning right')
%                         this.State = States.Stop;
                        this.Action = Actions.DodgeLeft;
                    elseif this.TaskDone == 1 || this.NoWall == 0
                        disp('turn to wall follow')
                        this.State = States.WallFollow;
                    else
                        this.Action = Actions.Task;
                    end
                case States.TurnLeft90
                    this.Action = Actions.Task;
                    if this.TaskDone == 1
                        disp('turn left done')
                        this.State = States.WallFollow;
                    end
                case States.TurnLeft2Explore
                    this.Action = Actions.Task;
                    if this.TaskDone == 1
                        disp('Turned, ready to Explore')
%                         this.State = States.Stop;
%                         Grid_rcd = this.grid_rcd
%                         stopPose = this.stopPoseWF
                        this.AddTasks(States.Explore);
                        this.State = States.Explore;
                    end
                case States.Forward2Explore
                    this.Action = Actions.Task;
                    if this.TaskDone == 1
                        disp('Forward aimed, ready to Explore')
                        this.AddTasks(States.Explore);
                        this.State = States.Explore;
                    end
                case States.Explore
                    this.Action = Actions.Task;
                    if this.BumpFront == 1 % ready for wall following
                        this.PosCalibBF()     % Calibrate Position when front bumped
                        this.WFDone = 0;    % clear Wall_Following Done flag
                        this.grid_rcd = []; % clear wall following grid record
%                         this.stopPoseWF = [6,3,2]; % !!!!!!!!!!for debug only!!!!!!!!
%                         disp('stop-pose changed')
                        this.AddTasks(States.TurnLeft90);
                        this.State = States.TurnLeft90;
                    end
                case States.Enter
                    this.Action = Actions.Task;
                    if this.TaskDone == 1
                        disp('entered, start wall following')
                        this.State = States.WallFollow;
                    end
            end
        end
        
        function AddTasks(this, state)
        % add tasks for different states to TaskSet
            switch state
                case States.TurnRight
                    this.TaskSet(1) = struct('type',1,'vel',0.2,'radiusTurn',[],'angle2Turn',[],'dist2Run',0.08,'target',[],'tol',0.001); %0.08 fine%0.10
                    this.TaskSet(2) = struct('type',2,'vel',0.1,'radiusTurn', -0.2,'angle2Turn', pi/2*2,'dist2Run',[],'target',[],'tol',0.001);
                    this.TaskInfo.num = 2;
                    this.TaskInfo.current = 1;
                    this.TaskDone = 0;
                    this.distRun = 0;           % Clear odometer for new task
                    this.angleTurn = 0;
                case States.TurnLeft90
                    % turn in place
                    this.TaskSet(1) = struct('type',2,'vel',0.1,'radiusTurn', 0,'angle2Turn', this.DestAngle(3),'dist2Run',[],'target',[],'tol',0.001);
                    this.TaskInfo.num = 1;
                    this.TaskInfo.current = 1;
                    this.TaskDone = 0;
                    this.distRun = 0;           % Clear odometer for new task
                    this.angleTurn = 0;
                case States.TurnLeft2Explore
                    % turn in place
                    this.TaskSet(1) = struct('type',2,'vel',0.1,'radiusTurn', 0,'angle2Turn', this.DestAngle(3),'dist2Run',[],'target',[],'tol',0.001); % Turn Left:3
                    this.TaskInfo.num = 1;
                    this.TaskInfo.current = 1;
                    this.TaskDone = 0;
                    this.distRun = 0;           % Clear odometer for new task
                    this.angleTurn = 0;
                case States.Forward2Explore
                    % aim ahead
                    this.TaskSet(1) = struct('type',2,'vel',0.1,'radiusTurn', 0,'angle2Turn', this.DestAngle(1),'dist2Run',[],'target',[],'tol',0.001); % Turn Left:3
                    this.TaskInfo.num = 1;
                    this.TaskInfo.current = 1;
                    this.TaskDone = 0;
                    this.distRun = 0;           % Clear odometer for new task
                    this.angleTurn = 0;
                case States.Explore
                    % Movig forward until front bumped
                    this.TaskSet(1) = struct('type',1,'vel',0.2,'radiusTurn',[],'angle2Turn',[],'dist2Run',this.gridLength*2,'target',[],'tol',0.001);% move more than one grid
                    this.TaskInfo.num = 1;
                    this.TaskInfo.current = 1;
                    this.TaskDone = 0;
                    this.distRun = 0;           % Clear odometer for new task
                    this.angleTurn = 0;
                case States.Enter
                    % Turn left (pi/2), move forward, turn right (-pi/2)
                    this.TaskSet(1) = struct('type',2,'vel',0.1,'radiusTurn', 0,'angle2Turn', pi/2,'dist2Run',[],'target',[],'tol',0.001);
                    this.TaskSet(2) = struct('type',1,'vel',0.2,'radiusTurn',[],'angle2Turn',[],'dist2Run',this.radius*2+0.025,'target',[],'tol',0.001); %0.08 fine%0.10
                    this.TaskSet(3) = struct('type',2,'vel',0.1,'radiusTurn', 0,'angle2Turn', -pi/2,'dist2Run',[],'target',[],'tol',0.001);
                    this.TaskInfo.num = 3;
                    this.TaskInfo.current = 1;
                    this.TaskDone = 0;
                    this.distRun = 0;           % Clear odometer for new task
                    this.angleTurn = 0;                    
            end
        end
        
        function [angle] = DestAngle(this, act_direct)
        % Compute turning angle based on direction action to turn(forward:1, left:3, etc.)
        % destination angle should be one of 0, 90, 180, 270 degrees
        % corresponding to Right:4, Up:1, Left:3, Down:2 directions
            % get destination direction
            switch act_direct
                case 1 % Action: Forward
                    dest_dir = this.Direct; % destination direction
                case 3 % Action: Left
                    switch this.Direct      % current direction
                        case 1              % Up
                            dest_dir = 3;   %     --> Left
                        case 2              % Down
                            dest_dir = 4;   %     --> Right
                        case 3              % Left
                            dest_dir = 2;   %     --> Down
                        case 4              % Right
                            dest_dir = 1;   %     --> Up
                    end
            end
            % get destination angle
            switch dest_dir
                case 1
                    angle = 1/2*pi;
                case 2
                    angle = 3/2*pi;
                case 3
                    angle = pi;
                case 4
                    angle = 0;
            end
            angle = mod(angle - this.theta, 2*pi);
            if angle > pi
                angle = mod(angle, pi) - pi;
            end
        end
        
        function value = get.Mode(this)
            % for debug
            value = 6;
            return
            % #
            if this.preMode ~= 0        % When previously not stop
                if this.BumpFront == 1 || this.NoWall == 1 || this.BumpRight == 1 || this.BumpLeft == 1
                    value = 0;          % then stop
                else
                    value = 1;
                end
            else                        % When previously stop
                if this.NoWall == 1 || this.BumpLeft == 1       % If no wall on right, turn right
                    value = 3; % Turn Right
                elseif this.BumpFront == 1 || this.BumpRight == 1 % If blocked ahead, turn left
                    value = 2;  % Turn left
                else
                    value = 1;  % Default: wall follower
                end
            end
            this.preMode = value;
            
        end
        function value = get.Direct(this)
        % Get one of four direction of robot based on theta    
%             angle = mod(this.theta, 2*pi);       % map angle to (0, 2*pi) 
            angle = this.theta;       % map angle to (0, 2*pi) 
            delta = 10/180*pi;
            divider = [ [1/4*pi+delta, 3/4*pi-delta];     % Up:     1
                        [5/4*pi+delta, 7/4*pi-delta];     % Down:   2
                        [3/4*pi+delta, 5/4*pi-delta];     % Left:   3
                        [           0, 1/4*pi-delta];    % Right1:  4
                        [7/4*pi+delta,         2*pi]];   % Right2:  4
            value = 0;
            for i = 1:length(divider)
                if (divider(i,1) <= angle && angle <= divider(i,2))
                    value = i;
                    break
                end
            end
            if value > 4    % value == 5 also means Right
                value = 4;
            end
        end
        
        PosCalibBF(this) % Calibrate position when bumped front
        [ new_mission ] = Plan( this, mission )       % Plan next cycle of wall following mission
        
    end
    
    methods(Static)
        [ stop, target, exploreDirect ] = Planning( map_grid, flow_map )
        [ stop, target, exploreDirect ] = PlanningReturn( outer_flow_rev, flow_map )
        [flow_map] = Flow2Map(flow_rcd, dimx, dimy)
        [ map_grid ] = MarkGridMap( flow_rcd, map_grid )
        PrintFlowMap( flow_map )
        GridMapPrint(map)
        [ ret ] = IsCovered(map_grid)
    end
    
end

