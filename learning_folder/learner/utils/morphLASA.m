function [X] =  morphLASA(T, n_morph, sigma)
   
    N = size(T,3);
    n = size(T,2);
    d = size(T,1);
    
    X = zeros(d,n,N * (n_morph + 1));
    idx = 1;
    %cb = jet(n_morph);
    
    for s = 1:N 
        %figure
        %add the orignal trajectory to new data set
        t = T(:,:,s);
        X(:,:,idx) = t;
        idx = idx + 1;
        %plot(t(1,:),t(2,:),'k');
        %hold on;
        for m = 1:n_morph
            tm = morphTrajectory(t, sigma);
            X(:,:,idx) = tm;
            idx = idx + 1;
            %plot(tm(1,:),tm(2,:),'Color',cb(m,:));
        end
        
        %axis([0,1,0,1]);
        %pause;
    end
end

