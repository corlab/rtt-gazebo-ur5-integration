function val = func_OmitRadius(x)
    [sX,sY] = size(x);
    val = zeros(sX,2);
    for i = 1:sX
        val(i,:) = [x(i,2),x(i,3)];
    end
end