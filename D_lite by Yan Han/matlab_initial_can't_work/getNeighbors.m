function neighborsList = getNeighbors(u, grid)
    [rows, cols] = size(grid);
    x = u(1);
    y = u(2);
    
    potentialNeighbors = [
        x-1, y;   % 上
        x+1, y;   % 下
        x, y-1;   % 左
        x, y+1;   % 右
    ];
    
    % 过滤掉超出边界的邻居
    validIndex = potentialNeighbors(:,1) > 0 & potentialNeighbors(:,1) <= rows & ...
                 potentialNeighbors(:,2) > 0 & potentialNeighbors(:,2) <= cols;
    neighborsList = potentialNeighbors(validIndex, :);
end