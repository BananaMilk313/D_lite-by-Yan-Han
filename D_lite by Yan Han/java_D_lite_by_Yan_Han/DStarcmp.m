function result = DStarcmp(a, b)
    % 比较两个数组 a 和 b
    % a 和 b 都是 double 类型的数组
    
    if a(3) < b(3) || (a(3) == b(3) && a(4) < b(4))
        result = -1;
    elseif a(3) > b(3) || (a(3) == b(3) && a(4) > b(4))
        result = 1;
    else
        result = 0;
    end
end