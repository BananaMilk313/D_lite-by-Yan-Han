function state = ResolvePath(state)
% ResolvePath 解析出最终的路径点列表
% 假设已从D* Lite结果中得到路径坐标 Xs, Ys（列向量或行向量）

    % 示例演示(请根据你的实际代码修改)
    % 假设Xs和Ys已在上层流程中求出
    Xs = [135; 134; 40; 17];  % 示例数据，请用你的数据替换
    Ys = [134; 133; 39; 16];  % 示例数据，请用你的数据替换

    % 确保 Xs 和 Ys 长度相同
    if length(Xs) ~= length(Ys)
        warning('ResolvePath: Xs and Ys have different lengths');
        state.path = [];
        return;
    end

    % 将路径点整理为 N x 2 矩阵
    path = [Xs(:), Ys(:)];

    state.path = path;
end
