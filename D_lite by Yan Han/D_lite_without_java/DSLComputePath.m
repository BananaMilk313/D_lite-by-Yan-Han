function outState = DSLComputePath(state, limit)
% D-star Lite implementation

    if nargin == 1
        limit = inf;
    end
    startPos = state.startPos;
    endPos = state.endPos;
    map = state.map;
    pattern = state.pattern;
    ucc = state.ucc;
    graph = state.graph;
    kM = state.kM;
    SQRT2 = sqrt(2) - 1;
    stack = state.stack;

    found = false;
    fprintf('DSLComputePath: startPos=(%d,%d), endPos=(%d,%d)\n', startPos(1), startPos(2), endPos(1), endPos(2));
    computeShortestPath();

    if found
        fprintf('DSLComputePath: Path found, rhs(startPos)=%.2f, g(startPos)=%.2f\n', rhs(startPos), g(startPos));
        setg(startPos, rhs(startPos));
    else
        fprintf('DSLComputePath: No path found.\n');
    end

    outState = state;
    outState.goal = startPos;
    outState.start = endPos;
    outState.graph = graph;
    outState.stack = stack;
    outState.length = graph(startPos(1), startPos(2), 2);

    % 打印统计信息
    all_g = graph(:,:,1);
    all_rhs = graph(:,:,2);
    fprintf('Number of visited nodes(g<inf): %d\n', sum(all_g(:)<inf));
    fprintf('Number of nodes with finite rhs: %d\n', sum(all_rhs(:)<inf));

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
        s = abs(startPos - s);
        out = SQRT2 * min(s(1:2)) + max(s(1:2));
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

    function computeShortestPath()
        terminate = false;
        count = 0;
        while ~terminate && size(stack,1) > 0
            count = count + 1;
            if count >= limit
                break;
            end
            u = stack(1,:);
            stack(1,:) = [];

            if ~( cmp(u(3:4), calculateKey(startPos)) || rhs(startPos) ~= g(startPos))
                found = true;
                break;
            end

            if ~inQ(u)
                continue;
            end

            rsetQ(u);
            gu = g(u);
            ru = rhs(u);

            if gu > ru
                setg(u, ru);
                gu = ru;
                for n = ucc
                    s = u + n;
                    if rhs(s) == inf % unvisited
                        if testNode(s)
                            setRhs(s, 1 + gu);
                            updatevertex(s);
                        else
                            setg(s, -1);
                            setRhs(s, -1);
                        end
                    else % visited
                        if rhs(s) > 1 + gu
                            setRhs(s, 1 + gu);
                            updatevertex(s);
                        end
                    end
                end
            end
        end
        fprintf('DSLComputePath: count=%d iterations\n', count);
    end

    function updatevertex(u)
        if g(u) ~= rhs(u)
            u(3:4) = calculateKey(u);
            add(u);
            setQ(u);
        else
            rsetQ(u);
        end
    end

    function out = testNode(s)
        out = true;
        for nn = pattern
            pos = s + nn;
            if map(pos(1), pos(2)) == 0
                out = false;
                return
            end
        end
    end

    function out = calculateKey(s)
        out = [
            min(g(s), rhs(s)) + heur(s) + kM;
            min(g(s), rhs(s))
        ];
    end

    function add(u)
        if isempty(stack)
            stack = u(:)';
        else
            inserted = false;
            for i = 1:size(stack,1)
                if cmp(u(3:4), stack(i,3:4))
                    stack = [stack(1:i-1,:); u(:)'; stack(i:end,:)];
                    inserted = true;
                    break;
                end
            end
            if ~inserted
                stack = [stack; u(:)'];
            end
        end
    end

    function out = cmp(s1, s2)
        out = s1(1) < s2(1) || (s1(1) == s2(1) && s1(2) < s2(2));
    end
end
