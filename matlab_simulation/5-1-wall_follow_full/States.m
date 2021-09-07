classdef (Sealed) States

properties  (Constant)
    Stop = 0;
    WallFollow = 1;
    Task = 2;
    TurnRight = 3;
    TurnLeft90 = 4;
    TurnLeft2Explore = 5; % After wall-following, turn left to explore un-visited cells
    Forward2Explore = 6;
    Explore = 7;
    Enter = 8;
    
end %constant properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
methods (Access = private)
%private so that you can't instatiate.
    function out = States
    end %Colors()
end %private methods

end %class Colors