% 定义环境
grid = [
    0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0;
    0 1 1 0 0 0 1 0 0 0;
    0 0 0 0 0 0 0 0 0 0;
    0 0 1 1 0 0 0 0 0 0;
    0 1 1 1 1 0 1 0 0 0;
    0 0 0 0 0 0 1 0 0 0;
    0 0 0 1 0 0 1 1 1 0;
    0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0;
];

start = [2, 2]; % 起点
goal = [9, 9];  % 终点

% 可视化环境
figure;
imagesc(grid);
colormap(gray);
hold on;
plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% 调用D_Lite算法
path = d_lite(grid, start, goal);

% 可视化路径
for i = 1:size(path, 1) - 1
    plot([path(i, 2), path(i+1, 2)], [path(i, 1), path(i+1, 1)], 'b-', 'LineWidth', 2);
    pause(0.2); % 动态显示路径
end

% D_Lite算法函数
