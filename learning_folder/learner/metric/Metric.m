classdef Metric < handle
    %METRIC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function m = Metric()
        end
        
        function d = distance(m, a, b)
            if size(b,1) < size(a,1)
                d = sqrt(sum((a-repmat(b,size(a,1),1)).^2,2));
            else
                d = sqrt(sum((a-b).^2,2));
            end
        end
    end
    
end

