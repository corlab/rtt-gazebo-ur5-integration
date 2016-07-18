function val = func_AddRadius(x,r)
    [sX,~] = size(x);
    val = zeros(sX,3);
    for i = 1:sX
        val(i,:) = [r,x(i,1),x(i,2)];
    end
end