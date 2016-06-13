function [result]=euler2dcm(v, verbose)

if (length(v)<3)
    
    if (verbose)
        error('euler2dcm() failed\n');
    end
    
    result=zeros(3);
    return;
end

Rza=eye(4,4);  Ryb=eye(4,4);   Rzg=eye(4,4);
alpha = v(1);  ca=cos(alpha);  sa=sin(alpha);
beta  = v(2);  cb=cos(beta);   sb=sin(beta);
gamma = v(3);  cg=cos(gamma);  sg=sin(gamma);

Rza(1,1)=ca;   Rza(2,2)=ca;   Rza(2,1)=sa;   Rza(1,2)=-sa;
Rzg(1,1)=cg;   Rzg(2,2)=cg;   Rzg(2,1)=sg;   Rzg(1,2)=-sg;
Ryb(1,1)=cb;   Ryb(3,3)=cb;   Ryb(3,1)=-sb;  Ryb(1,3)= sb;

result= Rza*Ryb*Rzg;
end