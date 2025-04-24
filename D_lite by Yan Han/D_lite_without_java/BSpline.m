function out = BSpline(points)
% 构建 B 样条插值。如果点数小于4个，则直接返回输入点。

    if size(points,1) < 4
        % 点数不足4个，无法构建B样条，直接返回原始点
        out = points;
        return;
    end

    n = 3;
    len = length(points(:,1));
    m = len - n;
    f = zeros(m,4);
    g = zeros(m,4);
    e = zeros(m+1,4);

    f(1,:) = points(2,:);
    g(1,:) = (points(2,:)+points(3,:))/2;

    for i = 2:m-1
        f(i,:) = (2*points(i+1,:)+points(i+2,:))/3;
        g(i,:) = (points(i+1,:)+2*points(i+2,:))/3;
    end

    f(m,:) = (points(m+3,:)+points(m+2,:))/2;
    g(m,:) = points(m+2,:);

    e(1,:) = points(1,:);
    for i = 2:1:m
        e(i,:) = (g(i-1,:)+f(i,:))/2;
    end
    e(m+1,:) = points(m+3,:);

    for i = 1:1:m
       BSplineC{i} =  [e(i,:); f(i,:); g(i,:); e(i+1,:)];
    end

    spline = [];
    t = 0:0.2:1;
    for i = 1:1:length(BSplineC)
        spline = [spline; bezier4(BSplineC{i}, t, 3)];
    end

    out = Interpolate(round(spline), 1, 'linear');
end
