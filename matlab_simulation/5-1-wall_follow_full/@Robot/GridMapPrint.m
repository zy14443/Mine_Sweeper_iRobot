function GridMapPrint(map)
    disp('----Grid Map:---')
    row_num = size(map,1);
    for row = row_num:-1:1
        disp(map(row,:))
    end
    disp('-----------------')
end
