function [R]= axis2dcm(v)

if length(v) < 4
    nv = norm(v);
    v = [v./nv; nv];
end

R=eye(4,4);

theta=v(4);
 
if (~theta)
return ;
end
c=cos(theta);
s=sin(theta);
C=1-c;

xs =v(1)*s;
ys =v(2)*s;
zs =v(3)*s;
xC =v(1)*C;
yC =v(2)*C;
zC =v(3)*C;
xyC=v(1)*yC;
yzC=v(2)*zC;
zxC=v(3)*xC;

R(1,1)=v(1)*xC+c;
R(1,2)=xyC-zs;
R(1,3)=zxC+ys;
R(2,1)=xyC+zs;
R(2,2)=v(2)*yC+c;
R(2,3)=yzC-xs;
R(3,1)=zxC-ys;
R(3,2)=yzC+xs;
R(3,3)=v(3)*zC+c;


end