function cc = func_FromSpericalCoords2CartesianCoords(pc)
    rad = pc(1,:);
    phi = pc(2,:);
    theta = pc(3,:);
    cc = [rad.*sin(theta).*cos(phi);rad.*sin(theta).*sin(phi);rad.*cos(theta)];
end