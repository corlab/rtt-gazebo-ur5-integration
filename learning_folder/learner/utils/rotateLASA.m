function [X] =  rotateLASA(T, n_rot)

    N=size(T,3);
    n=size(T,2);
    d=size(T,1);

    X = zeros(d,n,N * (n_rot + 1));
    i=1;
    for s=1:N 
        t = T(:,:,s);
        X(:,:,i) = t;
        i = i + 1;
        
        for r=1:n_rot
            alpha = rand * 2*pi;
            rot = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)];

            tr = rot*t;
            tr = normalizeTrajectory0to1(tr);
            X(:,:,i) = tr;
            i = i + 1;
        end
    end
end