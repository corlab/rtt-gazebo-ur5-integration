function N=normalizeTrajectory0to1(T)
    n = size(T,2);
    N = T - repmat(min(T,[],2),1,n);
    N = N/max(max(N));
end
