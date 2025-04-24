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