classdef (Sealed) Actions

properties  (Constant)
    Stop = 0;
    WallFollow = 1;
    TurnLeft = 2;
    TurnRight = 3;
    Task = 4;
    DodgeLeft = 5;
    DodgeRight = 6;
end %constant properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
methods (Access = private)
%private so that you can't instatiate.
    function out = Actions
    end %Colors()
end %private methods

end %class Colors