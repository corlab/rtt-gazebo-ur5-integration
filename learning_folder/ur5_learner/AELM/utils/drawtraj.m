function [negdata] = drawtraj(posx, posy, ink)

lengthscale=1;
xoffset = 5.5;
yoffset = 6; %%positive is downwards;

[numcases, maxtime] = size(posx);

images= zeros(36,36,numcases);
% NOTE: A digit is drawn in a 36x36 image.
%       Then a 28x28 image is cut out from its center.


%%now rescale and translate;
posx=posx*lengthscale + xoffset; 
posy=posy*lengthscale + yoffset;
for t=1:maxtime
    x=posx(:,t);
    y=posy(:,t);
    
    % discretize (x,y) coordinates so that they are on the pixel grid
    xlo=floor(x); xhi=ceil(x); 
    ylo=floor(y); yhi=ceil(y);
    xhi=xhi+(xlo==xhi);
    yhi=yhi+(ylo==yhi);
    
    wxlo=xhi-x; wxhi=x-xlo;
    wylo=yhi-y; wyhi=y-ylo;
    
    xlo=min(36,xlo); xlo=max(1,xlo); %%HACK: Put out of range ink in margins.
    ylo=min(36,ylo); ylo=max(1,ylo);
    xhi=min(36,xhi); xhi=max(1,xhi);
    yhi=min(36,yhi); yhi=max(1,yhi);
    
    loloinc=wylo.*wxlo;
    hiloinc=wyhi.*wxlo;
    lohiinc=wylo.*wxhi;
    hihiinc=wyhi.*wxhi;
    
    for nc=1:numcases
        images(ylo(nc),xlo(nc),nc)=images(ylo(nc),xlo(nc),nc)+2*loloinc(nc);
        images(yhi(nc),xlo(nc),nc)=images(yhi(nc),xlo(nc),nc)+2*hiloinc(nc);
        images(ylo(nc),xhi(nc),nc)=images(ylo(nc),xhi(nc),nc)+2*lohiinc(nc);
        images(yhi(nc),xhi(nc),nc)=images(yhi(nc),xhi(nc),nc)+2*hihiinc(nc);
    end;
end;



for t=2:maxtime  
    x = 0.25*posx(:,t) + 0.75*posx(:,t-1);
    y = 0.25*posy(:,t) + 0.75*posy(:,t-1);
    separation = sqrt((posx(:,t) - posx(:,t-1)).^2 + ...
        (posy(:,t) - posy(:,t-1)).^2 );
    intense = 2*max(0, min(1, separation-1)); %%rises from 0 to 1 as sep goes from 1 to 2
    xlo=floor(x); xhi=ceil(x); 
    ylo=floor(y); yhi=ceil(y);
    xhi=xhi+(xlo==xhi);
    yhi=yhi+(ylo==yhi);
    wxlo=xhi-x; wxhi=x-xlo;
    wylo=yhi-y; wyhi=y-ylo;
    xlo=min(36,xlo); xlo=max(1,xlo); %%HACK: Put out of range ink in margins.
    ylo=min(36,ylo); ylo=max(1,ylo);
    xhi=min(36,xhi); xhi=max(1,xhi);
    yhi=min(36,yhi); yhi=max(1,yhi);
    %fprintf(1,'%2.1f %2.1f %2.1f %2.1f \n',xlo(1),xhi(1),ylo(1),yhi(1));
    loloinc=wylo.*wxlo;
    hiloinc=wyhi.*wxlo;
    lohiinc=wylo.*wxhi;
    hihiinc=wyhi.*wxhi;
    for nc=1:numcases
        images(ylo(nc),xlo(nc),nc)=images(ylo(nc),xlo(nc),nc)+intense(nc)*loloinc(nc);
        images(yhi(nc),xlo(nc),nc)=images(yhi(nc),xlo(nc),nc)+intense(nc)*hiloinc(nc);
        images(ylo(nc),xhi(nc),nc)=images(ylo(nc),xhi(nc),nc)+intense(nc)*lohiinc(nc);
        images(yhi(nc),xhi(nc),nc)=images(yhi(nc),xhi(nc),nc)+intense(nc)*hihiinc(nc);
    end;
end;


for t=2:maxtime  
    x = 0.5*posx(:,t) + 0.5*posx(:,t-1);
    y = 0.5*posy(:,t) + 0.5*posy(:,t-1);
    separation = sqrt((posx(:,t) - posx(:,t-1)).^2 + ...
        (posy(:,t) - posy(:,t-1)).^2 );
    intense = 2*max(0, min(1, separation-1)); %%rises from 0 to 1 as sep goes from 1 to 2
    xlo=floor(x); xhi=ceil(x); 
    ylo=floor(y); yhi=ceil(y);
    xhi=xhi+(xlo==xhi);
    yhi=yhi+(ylo==yhi);
    wxlo=xhi-x; wxhi=x-xlo;
    wylo=yhi-y; wyhi=y-ylo;
    xlo=min(36,xlo); xlo=max(1,xlo); %%HACK: Put out of range ink in margins.
    ylo=min(36,ylo); ylo=max(1,ylo);
    xhi=min(36,xhi); xhi=max(1,xhi);
    yhi=min(36,yhi); yhi=max(1,yhi);
    %fprintf(1,'%2.1f %2.1f %2.1f %2.1f \n',xlo(1),xhi(1),ylo(1),yhi(1));
    loloinc=wylo.*wxlo;
    hiloinc=wyhi.*wxlo;
    lohiinc=wylo.*wxhi;
    hihiinc=wyhi.*wxhi;
    for nc=1:numcases
        images(ylo(nc),xlo(nc),nc)=images(ylo(nc),xlo(nc),nc)+intense(nc)*loloinc(nc);
        images(yhi(nc),xlo(nc),nc)=images(yhi(nc),xlo(nc),nc)+intense(nc)*hiloinc(nc);
        images(ylo(nc),xhi(nc),nc)=images(ylo(nc),xhi(nc),nc)+intense(nc)*lohiinc(nc);
        images(yhi(nc),xhi(nc),nc)=images(yhi(nc),xhi(nc),nc)+intense(nc)*hihiinc(nc);
    end;
end;

for t=2:maxtime  
    x = 0.75*posx(:,t) + 0.25*posx(:,t-1);
    y = 0.75*posy(:,t) + 0.25*posy(:,t-1);
    separation = sqrt((posx(:,t) - posx(:,t-1)).^2 + ...
        (posy(:,t) - posy(:,t-1)).^2 );
    intense = 2*max(0, min(1, separation-1)); 
    %%rises from 0 to 1 as sep goes from 1 to 2
    
    xlo=floor(x); xhi=ceil(x); 
    ylo=floor(y); yhi=ceil(y);
    xhi=xhi+(xlo==xhi);
    yhi=yhi+(ylo==yhi);
    wxlo=xhi-x; wxhi=x-xlo;
    wylo=yhi-y; wyhi=y-ylo;
    xlo=min(36,xlo); xlo=max(1,xlo); %%HACK: Put out of range ink in margins.
    ylo=min(36,ylo); ylo=max(1,ylo);
    xhi=min(36,xhi); xhi=max(1,xhi);
    yhi=min(36,yhi); yhi=max(1,yhi);
    %fprintf(1,'%2.1f %2.1f %2.1f %2.1f \n',xlo(1),xhi(1),ylo(1),yhi(1));
    loloinc=wylo.*wxlo;
    hiloinc=wyhi.*wxlo;
    lohiinc=wylo.*wxhi;
    hihiinc=wyhi.*wxhi;
    for nc=1:numcases
        images(ylo(nc),xlo(nc),nc)=images(ylo(nc),xlo(nc),nc)+intense(nc)*loloinc(nc);
        images(yhi(nc),xlo(nc),nc)=images(yhi(nc),xlo(nc),nc)+intense(nc)*hiloinc(nc);
        images(ylo(nc),xhi(nc),nc)=images(ylo(nc),xhi(nc),nc)+intense(nc)*lohiinc(nc);
        images(yhi(nc),xhi(nc),nc)=images(yhi(nc),xhi(nc),nc)+intense(nc)*hihiinc(nc);
    end;
end;


smallimages=zeros(28,28,numcases);
negdata = zeros(numcases,784);
bigimages=zeros(44,44,numcases); %%now allows big kernel


%%hinton(bigkernel);

%%kernel =  [4 6 4; 6 8 6; 4 6 4]/40;
%%kernel =  [1,4,1; 4,16,4; 1,4,1]/50;
for n=1:numcases
    th=ink(n,1)/2; %%controls width of convolution kernel.
    %%second entry in INK controls brightness.
    kernel = 1.5 * ink(n,2) * (1+th)* ...
        [th/12, th/6, th/12; th/6, 1-th, th/6;th/12, th/6, th/12];
    bigkernel = real(conv2(conv2(conv2(kernel,kernel),kernel),kernel)); 
    bigimages(:,:,n)=real(conv2(images(:,:,n), bigkernel));
    smallimages(:,:,n)=min(bigimages(9:36,9:36,n),1)';
    negdata(n,:)=reshape(smallimages(:,:,n),1,784);
end;