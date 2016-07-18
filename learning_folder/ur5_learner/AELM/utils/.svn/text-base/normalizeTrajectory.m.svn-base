function N=normalizeTrajectory(T)
    n = size(T,2);
    a = T(:,1);
    
    N = T - repmat(a,1,n);
    zn = N(:,end);
    alpha = atan2(zn(2),zn(1))-atan2(0,1);
    rotation = [cos(-alpha) -sin(-alpha); sin(-alpha) cos(-alpha)];
    rotAndScale = rotation/norm(zn);
    
    N = rotAndScale*N;
    %N = N - repmat([1;0],1,n);
end
