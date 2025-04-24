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

    % 获取路径
    path = state.path;

    % 如果path是cell类型，则先转换为数值矩阵
    if iscell(path)
        path = cell2mat(path);
    end

    % 确保为double类型
    path = double(path);

    if isempty(path)
        warning('PlotPath: No valid path found.');
        out.handle = [];
        out.path = [];
        return;
    end

    % 打印路径信息用于调试
    fprintf('PlotPath: Path has %d points.\n', size(path,1));
    disp('PlotPath: Path points sample (x y):');
    disp(path(1:min(end,5),:)); % 显示前5个点

    % 给路径增加第三、四列以满足后续处理要求
    path(:,3) = 1;
    path(:,4) = 1;

    % 如果未指定mapName则使用state.map
    if nargin < 3
        scalling = 1;
        imag = state.map;
    else
        tmp = LoadMap(strcat(mapName, '.png'), 1);
        imag = tmp.map;
    end

    imag = double(imag);

    % 如果需要缩放和平移
    if scalling ~= 1
        path(:,1) = path(:,1) - 2.5;
        path(:,2) = path(:,2) - 2.5;
        path = path * scalling;
    end

    fprintf('PlotPath: After scaling, path range: x=[%.2f, %.2f], y=[%.2f, %.2f]\n', ...
        min(path(:,1)), max(path(:,1)), min(path(:,2)), max(path(:,2)));

    % 如果点数少于4个，不使用BSpline
    if size(path,1) < 4
        pathInterpolated = path;
        fprintf('PlotPath: Points < 4, using original path.\n');
    else
        pathInterpolated = BSpline(path);
        fprintf('PlotPath: Using BSpline for smoothing.\n');
    end

    % 绘制路径
    for i = 1:size(pathInterpolated,1)
        x = round(pathInterpolated(i,1));
        y = round(pathInterpolated(i,2));
        if x > 0 && y > 0 && y <= size(imag,1) && x <= size(imag,2)
            imag(y, x) = 0.6;
        end
    end

    figure(100);
    out.handle = imshow(imag, []);
    out.path = pathInterpolated;
end
