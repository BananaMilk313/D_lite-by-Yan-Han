function out = DSLInit(startPos, endPos, map, scalling)
% DSLInit initializes the D* Lite algorithm with given parameters.
%
% Parameters:
%   startPos (vector): The starting position [x, y].
%   endPos (vector): The ending position [x, y].
%   map (matrix): The map matrix with obstacles.
%   scalling (double): The scaling factor for the map.
%
% Returns:
%   out (struct): The initialized state structure for D* Lite algorithm.

    %% Generate shape pattern
    radius = 10 / scalling;
    mat = zeros(radius * 2, radius * 2);
    dista = @(a, b) sqrt(a * a + b * b);

    shapePattern = [];

    for x = 1:2*radius
        for y = 1:2*radius
            if dista(x - radius - 0.5, y - radius - 0.5) > radius - 1 && dista(x - radius - 0.5, y - radius - 0.5) < radius
                mat(x, y) = 1;
                shapePattern = [shapePattern; floor(x - radius) floor(y - radius) 0 0];
            end
        end
    end
    
    neighbours = [
        0 1 0 0;
        -1 0 0 0;
        0 -1 0 0;
        1 1 0 0;
        -1 -1 0 0;
        1 0 0 0;
        1 -1 0 0;
        -1 1 0 0;
    ]';
    
    %% Prepare all data
    out.map = map;
    out.startPos = [startPos(2:-1:1); 0; 0];
    out.endPos = [endPos(2:-1:1); 0; 0];
    startPos = out.startPos;
      
    out.scalling = scalling;
    out.pattern = shapePattern';
    out.ucc = neighbours;
    out.height = ceil(length(out.map(:, 1)));
    out.width = ceil(length(out.map(1, :)));
    out.graph = zeros(out.height, out.width, 5);
    out.graph(:, :, 1:2) = inf;  % g-values and rhs-values initialized to infinity
    out.graph(:, :, 3) = false;  % Priority queue membership
    out.graph(:, :, 5) = -1;     % Additional data (e.g., parent pointers)
    out.kM = 0;
    SQRT2 = sqrt(2) - 1;

    % MATLAB-based priority queue: just a sorted list
    % We'll store states as rows of [x, y, key1, key2].
    out.stack = [];

    % Initialize the rhs-value of the end position
    setRhs(out.endPos, 0);
    setQ(out.endPos);
    out.endPos(3:4) = [heur(out.endPos); 0];
    add(out.endPos);

    %-----------------------------------------------------------
    function setQ(s)
        % Set the cell in the priority queue
        out.graph(s(1), s(2), 3) = true;
    end

    function setg(s, val)
        % Set the g-value of the cell
        out.graph(s(1), s(2), 1) = val;
    end

    function setRhs(s, val)
        % Set the rhs-value of the cell
        out.graph(s(1), s(2), 2) = val;
    end

    function val = heur(s)
        % Heuristic function (Manhattan distance)
        k = abs(startPos - s);
        val = SQRT2 * min(k(1:2)) + max(k(1:2));
    end

    % Priority queue operations
    function add(s)
        % Insert element s into out.stack sorted by priority using DStarcmp
        if isempty(out.stack)
            out.stack = s(:)';
        else
            inserted = false;
            for idx = 1:size(out.stack,1)
                if DStarcmp(s, out.stack(idx,:)) < 0
                    out.stack = [out.stack(1:idx-1,:); s(:)'; out.stack(idx:end,:)];
                    inserted = true;
                    break;
                end
            end
            if ~inserted
                out.stack = [out.stack; s(:)'];
            end
        end
    end

    function s = poll()
        % Remove and return element with highest priority (lowest keys)
        if isempty(out.stack)
            s = [];
        else
            s = out.stack(1,:);
            out.stack(1,:) = [];
        end
    end

    function tf = isEmpty()
        tf = isempty(out.stack);
    end

    % 如果后续代码(如DSLComputePath)需要访问add, poll, isEmpty等函数，
    % 可以在 out 中附加这些函数句柄，或在其他文件中复制这些逻辑。
    out.add = @add;
    out.poll = @poll;
    out.isEmpty = @isEmpty;
end
