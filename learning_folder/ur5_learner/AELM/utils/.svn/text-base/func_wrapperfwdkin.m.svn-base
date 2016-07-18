function [a3,b] = func_wrapperfwdkin(bha,lens)
    [sX,sY] = size(lens);
    for i = 1:sX
        [a(i,:),b(i,:)] = bha.fwdkinematics(lens(i,:));
        a2(i,:) = func_FromCartesianCoords2SpericalCoords(a(i,:));
        a3(i,:) = func_OmitRadius(a2(i,:));
    end
end