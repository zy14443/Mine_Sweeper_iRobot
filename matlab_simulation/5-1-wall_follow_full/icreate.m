function icreate(serPort)
% ICREATE Summary of this function goes here
% Detailed explanation goes here
    clc
    
    Duration = 430%56;%23; % total time duration %180 round:420+10s
    Freq10 = 10;                     % Frequency: 10Hz
    Freq20 = 20;                     % Frequency2: 20Hz for sensing
    target = 0.1;                   % Target distance to wall: 0.2 m
    % cte = ReadSonar(serPort, 1);    % right sonar reading, crosstrack error
    vel = 0.3;
    angl = 0;
    motion_params = {serPort, vel, angl, target};

    r = Robot(serPort);
    tc = TimerObj(Freq10);
    ts = TimerObj(Freq20);
    c = Controller(tc, r);
    s = Sensors(ts, r);
    
    pause(Duration+2)
    
    plot(r.x_rcd - 1.83, r.y_rcd - 1.83)
    plot(r.Pose_true_rcd(:,1), r.Pose_true_rcd(:,2), 'g') % plot true trajectory, in green
%     angl_rcd = c.angl_rcd
%     avg_angl_rcd = c.avg_angl_rcd
    figure
    plot(c.avg_angl_rcd)
    title('average angle record')
%    ;vg_ang_rcd
    figure
    plot(r.x_rcd,'k')
    hold on
    plot(r.y_rcd)
    title('x,y vs time')
    
    StopCreate(r.serPort)
    
    GridMapPrint(r.map_grid)
    Grid_Record = r.grid_rcd
    % -- delete objects --
    delete(r)
    delete(c)
    delete(s)
    delete(tc)
    delete(ts)

end

function StopCreate(serPort)
% Stop the robot
% serPort is the serial port number (for controlling the actual robot).
    SetDriveWheelsCreate(serPort, 0, 0)
end
function GridMapPrint(map)
    disp('----Grid Map:---')
    row_num = size(map,1);
    for row = row_num:-1:1
        disp(map(row,:))
    end
    disp('-----------------')
end