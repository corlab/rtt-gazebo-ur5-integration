function R = blendTrajectories(T1, T2)
    n = size(T1,2);
    d = size(T1,1);
    alpha = repmat(0:n-1,d,1)/(n-1);
    R = T2.*alpha + T1.*(1-alpha);
end