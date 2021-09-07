classdef (Sealed) Events

properties  (Constant)
    NoWall = 1;
    BumpFront = 2;
    BumpLeft = 4;
    BumpRight = 8;
    TaskDone = 16;
    WFDone = 32; % wall following finished
end %constant properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
methods (Access = private)
%private so that you can't instatiate.
    function out = Events
    end %Colors()
end %private methods

end %class Colors