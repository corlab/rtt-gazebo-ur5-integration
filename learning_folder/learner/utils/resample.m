function [Y] = resample(X, targetlength)
%RESAMPLE Summary of this function goes here
%   Detailed explanation goes here

if iscell(X)
    Y = cell(length(X),1);
    for i=1:length(X)
        currentlength = size(X{i},1);
        if currentlength == targetlength
            Y{i} = X{i};
        else
            Y{i} = zeros(targetlength,size(X{i},2));
            for j=1:size(X{i},2)
                Y{i}(:,j) = decimate(interp(X{i}(:,j), targetlength), currentlength);
            end
        end
    end
else
    currentlength = size(X,1);
    if currentlength == targetlength
        Y = X;
    else
        Y = zeros(targetlength,size(X,2));
        for j=1:size(X,2)
            Y(:,j) = decimate(interp(X(:,j), targetlength), currentlength);
        end
    end
end

end

