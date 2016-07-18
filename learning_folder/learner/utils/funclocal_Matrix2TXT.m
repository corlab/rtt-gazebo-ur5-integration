
function funclocal_matrix2txt(mat, name, FORCEMATRIX)
    fid = fopen(name,'wt');
    [sX,sY] = size(mat);
    
    if nargin<3
        FORCEMATRIX=false;
    end
    
    if (sX == 1 || sY == 1) && ~FORCEMATRIX
        str = sprintf('%d\n', max(sX,sY));
        fprintf(fid,str);

        for i = 1:max(sX,sY)
            str = sprintf('%16.16f ', mat(i));
            fprintf(fid,str);
        end
        fclose(fid); 
    else
        str = sprintf('%d\n%d\n', sX, sY);
        fprintf(fid,str);

        for i = 1:sX
            for j = 1:sY
                str = sprintf('%16.16f ', mat(i,j));
                fprintf(fid,str);
            end
            fprintf(fid,'\n');
        end
        fclose(fid); 
    end
end