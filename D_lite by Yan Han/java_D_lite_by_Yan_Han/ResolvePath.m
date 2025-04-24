function funOut = ResolvePath(state)
    % ResolvePath returns the resolved path from the goal to the start
    % using the D* Lite algorithm.
    %
    % Parameters:
    %   state (struct): The state structure containing the map, start, goal, and graph information.
    %
    % Returns:
    %   funOut (array): The resolved path from the goal to the start.

    funOut = resolve();

    % Function to get the g value of a node
    function out = g(s)
        out = state.graph(s(1), s(2), 1);
    end

    % Function to resolve the path
    function out = resolve()
        s = state.goal;
        out = [s];
        state.graph(s(1), s(2), 1) = inf;
        uval = g(s);
        minval = inf;
        
        % Loop until the goal is reached
        while ~isequal(s(1:2), state.start(1:2))
            minval = inf;
            it = [];
            
            % Iterate through neighbors
            for n = state.ucc
                u = s + n;
                uval = g(u);
                
                % Find the neighbor with the minimum g value
                if uval < minval && ~isinf(uval) && uval > -1
                    minval = uval;
                    it = n;
                end
            end
            
            % If no valid path is found, return
            if isempty(it)
                fprintf('There is no valid path\n');
                return
            end
            
            % Move to the next node in the path
            s = s + it;
            state.graph(s(1), s(2), 1) = inf;
            out = [out, s];
        end
    end
end