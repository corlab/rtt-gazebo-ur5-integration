function cc = func_FromPolarCoords2CartesianCoords(pc)
    rad = pc(1,:);
    phi = pc(2,:);
    cc = [rad.*cos(phi);rad.*sin(phi)];
end