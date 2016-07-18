function output = func_FromAngularCoords2CartesianCoords(input)
    [~,sY] = size(input);
    output = zeros(3,sY);
    
    for i = 1:sY
        inp = input(:,i);
        
        r = inp(1);
        aX = inp(2);
        aY = inp(3);

        x = cos(aX) * abs(r);
        y = cos(aY) * abs(r);
        rZ = max(0.0, r*r - x*x - y*y );
        z = sqrt(rZ) * sign(r);
        
        out = [x;y;z];
        
        output(:,i) = out;
    end
end