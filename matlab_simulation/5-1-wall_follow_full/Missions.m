classdef (Sealed) Missions

properties  (Constant)
    WFCircle = 0;       % Circle using wall following
    WF2Explore = 1;     % Wall follow to stop cell ready for explore
    WFReturn = 3;       % After all  cells covered, find a path out
    
end %constant properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
methods (Access = private)
%private so that you can't instatiate.
    function out = Missions
    end %Colors()
end %private methods

end %class Colors