clc;
clear;

% Initialize Java path (若已去除Java优先队列，可不需要此步骤)
clear java;
javaclasspath(pwd);

% Load maps
mapsNames = {'b', 'e', 'a', 'c', 'map01'};
maps = {};
% Scaling factor should be 1 (original map) or 4 (scaled map for better performance)
scalling = 4;
for i = 1:length(mapsNames)
    tmp = LoadMap(strcat(mapsNames{i}, '.png'), scalling);
    start = tmp.start';
    goal = tmp.goal';
    maps{end+1} = tmp.map;
end

% Initial search
clc;
close all;
tic;
state = DSLInit(start, goal, maps{1}, scalling);
state = DSLComputePath(state);
toc;

% 查看搜索后的路径信息
if isfield(state, 'path') && ~isempty(state.path)
    fprintf('Initial path found with %d points.\n', size(state.path,1));
else
    fprintf('No initial path found.\n');
end

% Update map and search in the updated map, run twice
state.kM = 50;
close all;
tic;
state = DSLUpdateMap(state, maps{2});
state = DSLComputePath(state);
toc;

% 再次检查路径
if isfield(state, 'path') && ~isempty(state.path)
    fprintf('Updated path found with %d points.\n', size(state.path,1));
else
    fprintf('No updated path found.\n');
end

% Resolve path
state.path = ResolvePath(state);
if isempty(state.path)
    fprintf('ResolvePath: No valid path resolved.\n');
else
    fprintf('ResolvePath: Path with %d points.\n', size(state.path,1));
end

% Plot path (with debugging information inside)
resp = PlotPath(state, scalling, 'e');

% Display D* graph (若需要)
figure;
a = state.graph(:,:,1);
a(a(:,:) ~= inf) = 0.3;
a(a(:,:) == inf) = 0;
a(state.map(:,:) == 0) = 0.2;
b = state.graph(:,:,3);
b(state.map(:,:) == 0) = 0;
b(state.graph(:,:,2) == inf) = 0;
b = b * 0.7;
imshow(b + a, 'Border', 'tight');
