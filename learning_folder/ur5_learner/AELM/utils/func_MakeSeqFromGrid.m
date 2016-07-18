
function seq = func_MakeSeqFromGrid(X,Y,Z)

    if nargin == 2
        [sX,sY] = size(X);
        seq = [];

        for i = 1:sX
            for j = 1:sY
                seq = [seq,[X(i,j);Y(i,j)]];
            end
        end
    else
        [sX,sY,sZ] = size(X);
        seq = [];

        for i = 1:sX
            for j = 1:sY
                for k = 1:sZ
                    seq = [seq,[X(i,j,k);Y(i,j,k);Z(i,j,k)]];
                end
            end
        end
    end
    
end