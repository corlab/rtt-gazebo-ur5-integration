function [X] =  blendLASA(T)
    N = size(T,3);
    n = size(T,2);

    X = zeros(2,n,N^2);

    k = 1;
    for i=1:N
        for j=1:N
            %if i~=j
                X(:,:,k) = blendTrajectories(T(:,:,i),T(:,:,j));
                k = k+1;
            %end
        end
    end
end