function phi = func_FromCartesianCoords2AngularCoords(x)
    [sX,sY] = size(x);
    x1 = [1;0;0]; x2 = [0;1;0]; 
    phi = zeros(sX,sY);
    for i = 1:sX
        vec = x(i,:);
        phi(i,:) = [sign(vec(3))*norm(vec), acos((vec*x1)/(norm(vec))), acos((vec*x2)/(norm(vec)))];
    end
end