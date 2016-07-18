function tm = morphTrajectory(t, sigma)
    n = size(t,2);
    d = size(t,1);
    min_centers = 3;
    max_centers = 5;
    n_centers = randInRange(min_centers, max_centers);
    centers = zeros(n_centers + 2,d,2);
    %start point
    centers(1,:,1)=1;
    centers(1,:,2)=0;
    %end point
    centers(end,:,1)=n;
    centers(end,:,2)=0;

    %compute 'n_centers' centers of displacement that will act as markers for
    %the spline interpolation
    lower = zeros(d,1);
    for k = 1:n_centers
        upper = floor(lower + ([n;n] - lower)./(n_centers - (k - 1)));
        c = zeros(d,1);
        for dim = 1:d
            %generate new center with 'sigma' noise, that (probably) keeps a certain
            %distance to the previous marker
            c(dim) = randInRange(lower(dim), upper(dim), true);
            centers(k+1,dim,1) = c(dim);
            centers(k+1,dim,2) = randn*sigma;
        end
        lower = c;
    end
    %generate displacement for each datapoint via spline
    %interpolation of centers
    noise = zeros(d,n);
    for dim=1:d
        noise(dim,:)=spline(centers(:,dim,1),centers(:,dim,2),1:n);
    end
    %perturb trajectory with noise
    tm = t + noise;
    %tm = normalizeTrajectory0to1(tm);
end