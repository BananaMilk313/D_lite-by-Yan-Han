% main.m
clc;
clear;

% Initialize Java path
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

% Update map and search in the updated map, run twice
state.kM = 50;
close all;
tic;
state = DSLUpdateMap(state, maps{2});
state = DSLComputePath(state);
toc;

% Resolve path
state.path = ResolvePath(state);
resp = PlotPath(state, scalling, 'e');

% Display D* graph
figure;
% Obstacles: 0.2, Visited: 0.3, Unavailable unvisited: 0, In queue: 1, In queue with g value as inf: 0.7
a = state.graph(:,:,1);
a(a(:,:) ~= inf) = 0.3;
a(a(:,:) == inf) = 0;
a(state.map(:,:) == 0) = 0.2;
b = state.graph(:,:,3);
b(state.map(:,:) == 0) = 0;
b(state.graph(:,:,2) == inf) = 0;
b = b * 0.7;
imshow(b + a, 'Border', 'tight');