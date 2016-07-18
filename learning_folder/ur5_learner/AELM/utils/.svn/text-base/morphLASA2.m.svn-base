function [X] =  morphLASA2(T, n_morph, sigma)

    maxMomentum = 0.9;
    minMomentum = 0.85;
    %M = [0.9 0.95 0.96];
    maxGravitation = 0.3;
    minGravitation = 0.01;
    %G = [0.01 0.03 0.5 0.1 0.15 0.2 0.3];
    
    N = size(T,3);
    n = size(T,2);
    d = size(T,1);
    %R = randperm(N);
    %T = T(:,:,R);
    
    X = zeros(d,n,N * (n_morph + 1));
    idx = 1;
    %cb = jet(n_morph);
    %figure
    for s = 1:N 
        %add the orignal trajectory to new data set
        t = T(:,:,s);
        X(:,:,idx) = t;
        idx = idx + 1;
        %hold off;
        %plot(t(1,:),t(2,:),'k.');
        %hold on;
        for i = 1:n_morph
            m = rand*(maxMomentum - minMomentum) + minMomentum;    
            g = rand*(maxGravitation - minGravitation) + minGravitation;        
            tm = morphTrajectory2(t, m, g, sigma);
            X(:,:,idx) = tm;
            idx = idx + 1;
            %plot(tm(1,:),tm(2,:),'Color',cb(i,:));
        end
        
        %{
        p = 1;
        for m = 1:numel(M)
            for g = 1:numel(G)       
                tm = morphTrajectory2(t, M(m), G(g), sigma);
                X(:,:,idx) = tm;
                idx = idx + 1;
                
                subplot(numel(M),numel(G),p);
                plot(t(1,:),t(2,:),'k');
                title(['m=' num2str(M(m)) '; g=' num2str(G(g))]);
                hold on;
                plot(tm(1,:),tm(2,:),'r');
                hold off;
                p = p + 1;
            end   
        end
        %}
        
        
        %axis([0,1,0,1]);
        %pause;
    end
end

