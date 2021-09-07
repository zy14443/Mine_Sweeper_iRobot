classdef (Sealed) Directs

properties  (Constant)
    Up = 1
    Forward = 1
    Down = 2
    Backward = 2
    Left = 3
    Right = 4
end %constant properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
methods (Access = private)
%private so that you can't instatiate.
    function out = Directs
    end %Colors()
end %private methods

end %class Colors