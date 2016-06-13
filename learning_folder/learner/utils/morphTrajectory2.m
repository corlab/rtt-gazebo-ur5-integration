function tm = morphTrajectory2(t, momentum, gc, sigma)
    n = size(t,2);
    d = size(t,1);
    tm = zeros(size(t));
    tm(:,1) = t(:,1) + randn(d,1)*sigma;
    vs = t(:,2:end) - t(:,1:end-1);
    v_m = vs(:,1);
    for i = 2:n
        g = t(:,i-1) - tm(:,i-1);
        v = vs(:,i-1);
        v_r = (1 - gc) * v + gc * g;
        
        v_m = momentum * v_m + (1 - momentum) * v_r;
        p = tm(:,i-1) +  v_m;
        tm(:,i) = p;
    end
    %tm = normalizeTrajectory0to1(tm);
end