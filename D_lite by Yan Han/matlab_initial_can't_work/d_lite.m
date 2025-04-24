function path = d_lite(grid, start, goal)
    % 初始化
    [rows, cols] = size(grid);
    U = PriorityQueue();
    g = inf(rows, cols);
    rhs = inf(rows, cols);
    g(goal(1), goal(2)) = 0;
    rhs(goal(1), goal(2)) = 0;
    U.insert(goal, calculateKey(goal, start, g, rhs));

    % 主循环
    iteration = 0;
    max_iterations = 10000; % 设置一个最大迭代次数以防止无限循环
    while ~U.isEmptyQueue() && iteration < max_iterations
        iteration = iteration + 1;
        [u, ~] = U.top();
        U.pop();

        % 打印调试信息
        fprintf('Iteration: %d, Current Node: (%d, %d), g: %f, rhs: %f\n', iteration, u(1), u(2), g(u(1), u(2)), rhs(u(1), u(2)));

        if g(u(1), u(2)) > rhs(u(1), u(2))
            g(u(1), u(2)) = rhs(u(1), u(2));
        else
            g(u(1), u(2)) = inf;
        end

        neighborsList = getNeighbors(u, grid);
        for i = 1:size(neighborsList, 1)
            s = neighborsList(i, :);
            if all(s > 0) && s(1) <= rows && s(2) <= cols && ~isequal(s, goal)
                rhs(s(1), s(2)) = min(rhs(s(1), s(2)), g(u(1), u(2)) + cost(u, s));
                if g(s(1), s(2)) ~= rhs(s(1), s(2))
                    U.insert(s, calculateKey(s, start, g, rhs));
                end
            end
        end
        if g(u(1), u(2)) ~= rhs(u(1), u(2))
            U.insert(u, calculateKey(u, start, g, rhs));
        end
    end

    if iteration >= max_iterations
        error('Exceeded maximum iterations. Possible infinite loop.');
    end

    % 生成路径
    path = [];
    current = start;
    while ~isequal(current, goal)
        path = [path; current];
        neighborsList = getNeighbors(current, grid);
        validNeighbors = neighborsList(all(neighborsList > 0, 2) & neighborsList(:, 1) <= rows & neighborsList(:, 2) <= cols & ...
                                       arrayfun(@(i) grid(neighborsList(i,1), neighborsList(i,2)) == 0, 1:size(neighborsList, 1))', :);
        if isempty(validNeighbors)
            error('No valid path found');
        end
        [~, idx] = min(arrayfun(@(i) g(validNeighbors(i, 1), validNeighbors(i, 2)), 1:size(validNeighbors, 1)));
        current = validNeighbors(idx, :);
    end
    path = [path; goal];
end

function key = calculateKey(node, start, g, rhs)
    % 计算键值
    h = heuristic(node, start);
    key = [min(g(node(1), node(2)), rhs(node(1), node(2))) + h, min(g(node(1), node(2)), rhs(node(1), node(2)))];
end

function h = heuristic(node, goal)
    % 使用曼哈顿距离作为启发式函数
    h = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end

function c = cost(node1, node2)
    % 假设移动成本为1
    c = 1;
end

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