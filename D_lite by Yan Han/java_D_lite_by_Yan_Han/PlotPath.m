function out = PlotPath(state, scalling, mapName)
% PlotPath uses B-spline interpolation to plot the path on the map.
%
% Parameters:
%   state (struct): The state structure containing the map and path information.
%   scalling (double): The scaling factor for the map and path.
%   mapName (string): The name of the map file (optional).
%
% Returns:
%   out (struct): A structure containing the handle to the plotted image and the interpolated path.

    % Extract the path from the state and add additional columns for B-spline interpolation
    path = state.path';
    path(:,3) = 1;
    path(:,4) = 1;

    % If mapName is not provided, use the map from the state
    if nargin < 3
        scalling = 1;
        imag = state.map;
    else
        tmp = LoadMap(strcat(mapName, '.png'), 1);
        imag = tmp.map;
    end

    % Adjust the path coordinates based on the scaling factor
    if scalling ~= 1
        path(:,1) = path(:,1) - 2.5;
        path(:,2) = path(:,2) - 2.5;
        pathInterpolated = BSpline(path * scalling);
    else
        pathInterpolated = path;
    end

    % Mark the interpolated path on the map
    for it = pathInterpolated'
        a = round(it);
        imag(a(1), a(2)) = 0.6;
    end

    % Plot the map with the path
    figure(100)
    out.handle = imshow(imag);
    out.path = pathInterpolated;
end