function PosCalibBF(this)
% Position Calibration when Front Bump activated
    if this.BumpFront == 0
        return
    end
    if this.Direct == 1         % When moving UP
        delta = 0;
        area_base = [0.5 + delta, 1.5 - delta];
        frontEdge_y = this.y + this.radius; % Front edge position
        if frontEdge_y < area_base(2) * this.gridLength                          % 1st bar
            this.y = this.gridLength * 1 - this.radius; % Center position

        elseif (area_base(1)+1) * this.gridLength < frontEdge_y &&...            % 2nd bar
                             frontEdge_y < (area_base(2)+1) * this.gridLength
            this.y = this.gridLength * 2 - this.radius; % Center position

        elseif (area_base(1)+2) * this.gridLength < frontEdge_y &&...            % 3rd bar
                             frontEdge_y < (area_base(2)+2) * this.gridLength
            this.y = this.gridLength * 3 - this.radius; % Center position

        elseif (area_base(1)+3) * this.gridLength < frontEdge_y &&...            % 4th bar
                             frontEdge_y < (area_base(2)+3) * this.gridLength
            this.y = this.gridLength * 4 - this.radius; % Center position

        elseif (area_base(1)+4) * this.gridLength < frontEdge_y &&...            % 5th bar
                             frontEdge_y < (area_base(2)+4) * this.gridLength
            this.y = this.gridLength * 5 - this.radius; % Center position

        elseif frontEdge_y > (area_base(2)+4) * this.gridLength                  % 6th bar
            this.y = this.gridLength * 6 - this.radius; % Center position                       
        end
        fprintf(1, 'UP: y changed, y = %f\n', this.y)
%                 for row = 0:(this.gridSize - 1)     % must be in one of the six area rows
%                     area = area_base + row;         % row-th area
%                     if area(1) * this.gridLength < frontEdge_y &&...
%                                        frontEdge_y < area(2) * this.gridLength
%                         this.y = this.gridLength * (row + 1) - this.radius; % Center position
%                         fprintf(1, 'y changed, y = %f\n', this.y)
%                         break
%                     end
%                 end
%                 % when out of two boundaries
%                 if frontEdge_y < area_base(1) * this.gridLength
%                     this.y = this.gridLength *  1 - this.radius;  % lowest bar
%                 elseif frontEdge_y > (area_base(2) + (this.gridSize - 1)) * this.gridLength
%                     this.y = this.gridLength *  this.gridSize - this.radius; % highest bar
%                 end
    elseif this.Direct == 2     % When moving Down
        delta = 0;
        area_base = [0.5 + delta, 1.5 - delta];
        frontEdge_y = this.y - this.radius; % Front edge downwards
        if frontEdge_y < area_base(1) * this.gridLength                          % 0th bar
            this.y = this.gridLength * 0 + this.radius; % Center position

        elseif area_base(1) * this.gridLength < frontEdge_y &&...                % 1st bar
                             frontEdge_y < area_base(2) * this.gridLength
            this.y = this.gridLength * 1 + this.radius; % Center position

        elseif (area_base(1)+1) * this.gridLength < frontEdge_y &&...            % 2nd bar
                             frontEdge_y < (area_base(2)+1) * this.gridLength
            this.y = this.gridLength * 2 + this.radius; % Center position

        elseif (area_base(1)+2) * this.gridLength < frontEdge_y &&...            % 3rd bar
                             frontEdge_y < (area_base(2)+2) * this.gridLength
            this.y = this.gridLength * 3 + this.radius; % Center position

        elseif (area_base(1)+3) * this.gridLength < frontEdge_y &&...            % 4th bar
                             frontEdge_y < (area_base(2)+3) * this.gridLength
            this.y = this.gridLength * 4 + this.radius; % Center position

        elseif frontEdge_y > (area_base(1)+4) * this.gridLength                  % 5th bar
            this.y = this.gridLength * 5 + this.radius; % Center position                       
        end
        fprintf(1, 'Down: y changed, y = %f\n', this.y)                
    elseif this.Direct == 3     % When moving Left 
        delta = 0;
        area_base = [0.5 + delta, 1.5 - delta];
        frontEdge_x = this.x - this.radius; % Front edge Leftwards
        if frontEdge_x < area_base(1) * this.gridLength                          % 0th bar
            this.x = this.gridLength * 0 + this.radius; % Center position

        elseif area_base(1) * this.gridLength < frontEdge_x &&...                % 1st bar
                             frontEdge_x < area_base(2) * this.gridLength
            this.x = this.gridLength * 1 + this.radius; % Center position

        elseif (area_base(1)+1) * this.gridLength < frontEdge_x &&...            % 2nd bar
                             frontEdge_x < (area_base(2)+1) * this.gridLength
            this.x = this.gridLength * 2 + this.radius; % Center position

        elseif (area_base(1)+2) * this.gridLength < frontEdge_x &&...            % 3rd bar
                             frontEdge_x < (area_base(2)+2) * this.gridLength
            this.x = this.gridLength * 3 + this.radius; % Center position

        elseif (area_base(1)+3) * this.gridLength < frontEdge_x &&...            % 4th bar
                             frontEdge_x < (area_base(2)+3) * this.gridLength
            this.x = this.gridLength * 4 + this.radius; % Center position

        elseif frontEdge_x > (area_base(1)+4) * this.gridLength                  % 5th bar
            this.x = this.gridLength * 5 + this.radius; % Center position                       
        end
        fprintf(1, 'Left: x changed, x = %f\n', this.x)                          
    elseif this.Direct == 4     % When moving Right
        delta = 0;
        area_base = [0.5 + delta, 1.5 - delta];
        frontEdge_x = this.x + this.radius; % Front edge towards right
        if frontEdge_x < area_base(2) * this.gridLength                          % 1st bar
            this.x = this.gridLength * 1 - this.radius; % Center position

        elseif (area_base(1)+1) * this.gridLength < frontEdge_x &&...            % 2nd bar
                             frontEdge_x < (area_base(2)+1) * this.gridLength
            this.x = this.gridLength * 2 - this.radius; % Center position

        elseif (area_base(1)+2) * this.gridLength < frontEdge_x &&...            % 3rd bar
                             frontEdge_x < (area_base(2)+2) * this.gridLength
            this.x = this.gridLength * 3 - this.radius; % Center position

        elseif (area_base(1)+3) * this.gridLength < frontEdge_x &&...            % 4th bar
                             frontEdge_x < (area_base(2)+3) * this.gridLength
            this.x = this.gridLength * 4 - this.radius; % Center position

        elseif (area_base(1)+4) * this.gridLength < frontEdge_x &&...            % 5th bar
                             frontEdge_x < (area_base(2)+4) * this.gridLength
            this.x = this.gridLength * 5 - this.radius; % Center position

        elseif frontEdge_x > (area_base(2)+4) * this.gridLength                  % 6th bar
            this.x = this.gridLength * 6 - this.radius; % Center position                       
        end
        fprintf(1, 'Right: x changed, x = %f\n', this.x)
%                 for col = 0:(this.gridSize - 1)     % must be in one of the six area columns
%                     area = area_base + col;         % col-th area
%                     if area(1) * this.gridLength < frontEdge_x &&...
%                                        frontEdge_x < area(2) * this.gridLength
%                         this.x = this.gridLength * (col + 1) - this.radius; % Center position
%                         fprintf(1, 'x changed, x = %f\n', this.x)
%                         break
%                     end
%                 end
    end

end