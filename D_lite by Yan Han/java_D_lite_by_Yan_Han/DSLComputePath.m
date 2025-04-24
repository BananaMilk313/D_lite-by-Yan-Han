function outState = DSLComputePath(state, limit)
% D-star Lite implementation
%
% Parameters:
%   state (struct): The state structure containing the map and other parameters.
%   limit (int): The maximum number of iterations for the path computation.
%
% Returns:
%   outState (struct): The updated state structure after path computation.

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
    computeShortestPath();

    if found
        fprintf('Found!: %d\n', graph(startPos(1), startPos(2), 2));
        setg(startPos, rhs(startPos));
    end

    outState = state;
    outState.goal = startPos;
    outState.start = endPos;
    outState.graph = graph;
    outState.stack = stack;
    outState.length = graph(startPos(1), startPos(2), 2);

    % Function to plot positions
    function plotPos(s)
        for n = pattern
            pos = s + n';
            if isInRange(pos)
                outImg(pos(1), pos(2)) = 1;
            end
        end
    end

    % Function to check if a node is in the priority queue
    function out = inQ(s)
        out = graph(s(1), s(2), 3);
    end

    % Function to set a node in the priority queue
    function setQ(s)
        graph(s(1), s(2), 3) = true;
    end

    % Function to remove a node from the priority queue
    function rsetQ(s)
        graph(s(1), s(2), 3) = false;
    end

    % Function to increment a node's value
    function incr(s)
        graph(s(1), s(2), 4) = graph(s(1), s(2), 4) + 1;
    end

    % Heuristic function (Manhattan distance)
    function out = heur(s)
        s = abs(startPos - s);
        out = SQRT2 * min(s(1:2)) + max(s(1:2));
    end

    % Function to get the g-value of a node
    function out = g(s)
        out = graph(s(1), s(2), 1);
    end

    % Function to set the g-value of a node
    function setg(s, val)
        graph(s(1), s(2), 1) = val;
    end

    % Function to get the rhs-value of a node
    function out = rhs(s)
        out = graph(s(1), s(2), 2);
    end

    % Function to set the rhs-value of a node
    function setRhs(s, val)
        graph(s(1), s(2), 2) = val;
    end

    % Function to calculate the key for the priority queue
    function out = calculateKey(s)
        out = [
            min(g(s), rhs(s)) + heur(s) + kM;
            min(g(s), rhs(s))
        ];
    end

    % Function to test if a node is available
    function out = testNode(s)
        out = true;
        for n = pattern
            pos = s + n;
            if map(pos(1), pos(2)) == 0
                out = false;
                return
            end
        end
    end

    % Function to compare keys (s1 > s2)
    function out = cmp(s1, s2)
        out = s1(1) < s2(1) || (s1(1) == s2(1) && s1(2) < s2(2));
    end

    % Function to update a vertex
    function updatevertex(u)
        if g(u) ~= rhs(u)
            u(3:4) = calculateKey(u);
            add(stack, u);
            setQ(u);
        else
            rsetQ(u);
        end
    end

    % Function to compute the shortest path
    function computeShortestPath()
        terminate = false;
        count = 0;

        while ~terminate && size(stack) > 0
            count = count + 1;
            if count >= limit
                break;
            end
            u = remove(stack);

            if ~(cmp(u(3:4), calculateKey(startPos)) || rhs(startPos) ~= g(startPos))
                found = true;
                break;
            end

            if ~inQ(u)
                continue;
            end % if node shouldn't be in queue

            if isequal(u(1:2), startPos(1:2))
                found = true;
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
        fprintf('count: %d\n', count);
    end
end