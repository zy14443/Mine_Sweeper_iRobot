function [flow_map] = Flow2Map(flow_rcd, dimx, dimy)
% FLOW2MAP mark flow to the map
% input: flow sequence, dimensions of the map
    flow_map(1:dimx,1:dimy) = struct('Up',-1,'Down',-1,'Left',-1,'Right',-1);
    rcd_num = size(flow_rcd, 1);
    for i = 1 : rcd_num
        switch flow_rcd(i,3)
            case 1
                flow_map(flow_rcd(i,2), flow_rcd(i,1)).Up = i;
            case 2
                flow_map(flow_rcd(i,2), flow_rcd(i,1)).Down = i;
            case 3
                flow_map(flow_rcd(i,2), flow_rcd(i,1)).Left = i;
            case 4
                flow_map(flow_rcd(i,2), flow_rcd(i,1)).Right = i;
        end
     end
 
end






