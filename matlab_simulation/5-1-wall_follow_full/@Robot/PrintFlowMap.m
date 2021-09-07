function PrintFlowMap( flow_map )
%PRINTFLOWMAP Print out the flow map
    [dimy, dimx] = size(flow_map);
    disp('----Flow Map:----')
    for i = dimy:-1:1
        for j = 1:dimx
            grid = flow_map(i, j);
            fprintf(1,'^%2dv%2d<%2d>%2d  ',grid.Up, grid.Down, grid.Left, grid.Right);
        end
        fprintf(1,'\n\n')
%         for j = 1:dimx
%             fprintf(1,'|  v -> <- ');
%         end
%         fprintf(1,'\n')
    end
    disp('----------------')

end

