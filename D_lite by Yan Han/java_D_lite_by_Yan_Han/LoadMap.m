function out = LoadMap(name, scalling)
    % LoadMap returns a structure containing the map, start, and goal positions.
    % The map can be rescaled based on the provided scaling factor.
    %
    % Parameters:
    %   name (string): The name of the map file to be loaded.
    %   scalling (int): The scaling factor for the map. Default is 1.
    %
    % Returns:
    %   out (struct): A structure containing the rescaled map, start position, and goal position.

    if nargin == 1
        scalling = 1; % Default scaling factor is 1 if not provided
    end
    
    % Call segmentation function to get robot and target positions and the map
    [robot_xy, target_xy, map] = segmentation(name, 0.1, 0.9, 0.5);
    
    % Rescale the map if the scaling factor is not 1
    if scalling ~= 1
        map = simplifyMap(map, scalling);
    end
    
    % Get the size of the map
    [a, b] = size(map);
    
    % Initialize the output map with padding
    out.map = zeros(a + 10, b + 10);
    out.map(5:end-6, 5:end-6) = map;
    
    % Calculate the start and goal positions with scaling and padding
    out.start = floor(robot_xy / scalling) + [5, 5];
    out.goal = floor(target_xy / scalling) + [5, 5];
end