function outState = DSLUpdateMap(state, newMap)
% DSLUpdateMap updates the map and re-computes the path using D* Lite algorithm.
%
% Parameters:
%   state (struct): The current state structure containing the map and path information.
%   newMap (matrix): The new map matrix with updated obstacles.
%
% Returns:
%   outState (struct): The updated state structure with the new map and graph information.

    startPos = state.startPos;
    endPos = state.endPos;
    pattern = state.pattern;
    ucc = state.ucc;
    graph = state.graph;
    kM = state.kM;
    SQRT2 = sqrt(2) - 1;

    map = newMap;    
    stack = state.stack;   % 原有的stack（主优先队列）
    stack2 = [];           % 辅助优先队列2
    stack3 = [];           % 辅助优先队列3
    
    difference = map - state.map;
    state.map = map;

    %==================== 处理新增的障碍 ====================
    [x, y] = find(difference == -1);
    indices = [x, y];
    indices(:, 3:4) = 0;      % 确保形成 [x y 0 0] 格式
    Ind = indices';

    for n = indices'
        for s = pattern
            Ind = [Ind, (n + s)]; % 改动点：去掉转置'
        end
    end
    added = unique(Ind', 'rows')';

    %==================== 处理移除的障碍 ====================
    [x, y] = find(difference == 1);
    indices = [x, y];
    indices(:, 3:4) = 0;
    Ind = indices';

    for n = indices'
        for s = pattern
            Ind = [Ind, (n + s)]; % 改动点：去掉转置'
        end
    end
    removed = unique(Ind', 'rows')';

    % 处理removed（原本是减少的障碍，即该处原有区域现在可通过）
    for u = removed
        tmp = inf;
        for n = ucc
            s = u + n;
            if map(s(1), s(2)) ~= 0 && rhs(s) ~= -1 && test(s)
                tmp = min(tmp, 1 + rhs(s));
            end
        end
        setRhs(u, tmp);
        setg(u, inf);
        u(3:4) = calculateKey(u);
        addMain(u);
        setQ(u);
    end

    % 处理added（新增加的障碍）
    if ~isempty(added)
        a = graph(:, :, 1);
        b = graph(:, :, 2);
        c = graph(:, :, 3);
        d = c; 
        d(:, :) = 0;

        a(sub2ind(size(a), added(1, :), added(2, :))) = inf;
        b(sub2ind(size(a), added(1, :), added(2, :))) = inf;
        c(sub2ind(size(a), added(1, :), added(2, :))) = 0;

        graph(:, :, 1) = a;
        graph(:, :, 2) = b;

        cnt = [];
        for u = added
            for n = ucc
                s = u + n;
                if rhs(s) < rhs(u) && ~isinf(rhs(s))
                    cnt = [cnt, s];
                end
            end
        end

        c(sub2ind(size(c), cnt(1, :), cnt(2, :))) = 1;

        % Identify cells that lost connection
        for u = cnt
            hasNbr = false;
            for n = ucc
                s = u + n;
                if ~inQ(s) && g(s) < g(u) && ~isinf(rhs(s))
                    hasNbr = true;
                end
            end

            if ~hasNbr
                u(3:4) = [g(u), 0];
                add3(u);
                rsetQ(u);
            end
        end

        clearChilds();

        a = graph(:, :, 1);
        b = graph(:, :, 2);
        c = graph(:, :, 3);

        a(a(:, :) ~= inf) = 1;
        a(a(:, :) == inf) = 0;
        b(b(:, :) ~= inf) = 1;
        b(b(:, :) == inf) = 0;
        dif = b - a;
        graph(:, :, 3) = graph(:, :, 3) + dif;
    end

    outState = state;
    outState.graph = graph;
    outState.map = map;
    outState.stack = stack;

    %%%%%%%%%%%%%%%%%%%%% 内部函数区 %%%%%%%%%%%%%%%%%%%%%

    function out = test(s)
        % Test if all neighboring cells are available
        out = true;
        for nn = pattern
            pos = s + nn;
            if map(pos(1), pos(2)) == 0
                out = false;
                return
            end
        end
    end

    function out = inQ(s)
        out = graph(s(1), s(2), 3);
    end

    function setQ(s)
        graph(s(1), s(2), 3) = true;
    end

    function rsetQ(s)
        graph(s(1), s(2), 3) = false;
    end

    function out = heur(s)
        k = abs(startPos - s);
        out = SQRT2 * min(k(1:2)) + max(k(1:2));
    end

    function out = g(s)
        out = graph(s(1), s(2), 1);
    end

    function setg(s, val)
        graph(s(1), s(2), 1) = val;
    end

    function out = rhs(s)
        out = graph(s(1), s(2), 2);
    end

    function setRhs(s, val)
        graph(s(1), s(2), 2) = val;
    end

    function out = calculateKey(s)
        out = [ min(g(s), rhs(s)) + heur(s) + kM; min(g(s), rhs(s)) ];
    end

    % cmp函数用于比较两个键值(key)的优先级
    % 返回true当 s1 < s2
    function out = cmp(s1, s2)
        out = s1(1) < s2(1) || (s1(1) == s2(1) && s1(2) < s2(2));
    end

    % 主优先队列(stack)的操作
    function addMain(u)
        if isempty(stack)
            stack = u(:)';
            return;
        end
        inserted = false;
        for i = 1:size(stack,1)
            if cmp(u(3:4), stack(i, 3:4))
                stack = [stack(1:i-1,:); u(:)'; stack(i:end,:)];
                inserted = true;
                break;
            end
        end
        if ~inserted
            stack = [stack; u(:)'];
        end
    end

    function u = removeMain()
        u = stack(1,:);
        stack(1,:) = [];
    end

    % stack2的操作
    function add2(u)
        if isempty(stack2)
            stack2 = u(:)';
            return;
        end
        inserted = false;
        for i = 1:size(stack2,1)
            if cmp(u(3:4), stack2(i, 3:4))
                stack2 = [stack2(1:i-1,:); u(:)'; stack2(i:end,:)];
                inserted = true;
                break;
            end
        end
        if ~inserted
            stack2 = [stack2; u(:)'];
        end
    end

    function u = remove2()
        u = stack2(1,:);
        stack2(1,:) = [];
    end

    % stack3的操作
    function add3(u)
        if isempty(stack3)
            stack3 = u(:)';
            return;
        end
        inserted = false;
        for i = 1:size(stack3,1)
            if cmp(u(3:4), stack3(i, 3:4))
                stack3 = [stack3(1:i-1,:); u(:)'; stack3(i:end,:)];
                inserted = true;
                break;
            end
        end
        if ~inserted
            stack3 = [stack3; u(:)'];
        end
    end

    function u = remove3()
        u = stack3(1,:);
        stack3(1,:) = [];
    end

    % 清理子节点函数
    function clearChilds()
        % 在处理added障碍前已经定义过d = c; d(:,:) = 0;
        % 确保此处可以访问d，否则需要根据实际情况调整
        
        while size(stack3,1) > 0
            u = remove3();
            g_old = g(u);
            setg(u, inf);
            setRhs(u, inf);
            rsetQ(u);
            for n = ucc
                s = u + n;
                if d(s(1), s(2)) == 0 && ~isinf(rhs(s)) && rhs(s) > g_old && map(s(1), s(2)) ~= 0
                    d(s(1), s(2)) = 1;
                    s(3:4) = [g(s), 0];
                    add3(s);
                    rsetQ(s);
                elseif ~isinf(g(s)) && g(s) == g_old - 1 && map(s(1), s(2)) ~= 0
                    add2(s);
                    setg(s, inf);
                end
            end
        end

        % 处理stack2
        while size(stack2,1) > 0
            u = remove2();
            for n = ucc
                s = u + n;
                if ~isinf(g(s))
                    u(3:4) = [rhs(u), 0];
                    addMain(u);
                    setQ(u);
                    break;
                end
            end
        end
    end
end
