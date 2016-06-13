function [v]=dcm2euler(R, verbose)
%v=zeros(3,1);
if nargin < 2
    verbose = 0;
end
if (size(R,1)<3 || size(R,2)<3)
    'test'
    if (verbose)
        error('dcm2euler() failed\n');
    end
    return
end


r2 = R(3,1)*R(3,1) + R(3,2)*R(3,2);
if (r2 > 0)
    
    v(2)=atan2(sqrt(r2), R(3,3));
    v(1)=atan2(R(2,3)/sin(v(2)), R(1,3)/sin(v(2)));
    v(3)=atan2(R(3,2)/sin(v(2)),-R(3,1)/sin(v(2)));
    
else
    
    if (verbose)
        disp('dcm2euler() in singularity: choosing one solution among multiple\n');
    end
    v(2)=0;
    v(1)=atan2(R(2,1), R(1,1));
    v(3)=0;
end


end

