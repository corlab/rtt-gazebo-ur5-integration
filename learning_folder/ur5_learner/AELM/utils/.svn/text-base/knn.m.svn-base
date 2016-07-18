function [Wk indices distances] = knn(X, w, K)
%KNN K-nearest neighbor search/vector quantization
%   [Wk indices distances] = knn(X, w, K)

D = sum((X - repmat(w, size(X,1), 1)).^2, 2);
[distances indices] = sort(D, 1, 'ascend');
distances = distances(1:min(K,size(distances,1)));
indices = indices(1:min(K,size(indices,1)));
Wk = X(indices,:);

end

