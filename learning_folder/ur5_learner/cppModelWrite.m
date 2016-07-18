function [] = cppModelWrite( elm_learner , massidx )
%CPPMODELWRITE Summary of this function goes here
%   Detailed explanation goes here
%Save ELM to file for import in c++  (maybe move into elm class ?!):


if massidx == 1
    target = '../elmmodel_mass_0';
elseif massidx == 2
    target = '../elmmodel_mass_1';
elseif massidx == 3
    target = '../elmmodel_mass_5';
elseif massidx == 4
    target = '../elmmodel_mass_3';    
else 
    target = '../elmmodel';
end




datatarget = sprintf('%s/data/',target);

mkdir(target);
mkdir(datatarget);

%Write matrix data
%Ax = diag(2./(elm_learner.inpRange(2,:)-elm_learner.inpRange(1,:)));
Ax = diag( 1./(elm_learner.inpRange(2,:)-elm_learner.inpRange(1,:)));
bx = elm_learner.inpOffset(1,:)+elm_learner.inpRange(1, :);
%bx = -(2./(elm_learner.inpRange(2,:)-elm_learner.inpRange(1,:))).*(elm_learner.inpOffset + elm_learner.inpRange(1,:)) - 1;
%Ay = diag(2./(elm_learner.outRange(2,:)-elm_learner.outRange(1,:)));
Ay = diag(elm_learner.outRange(2,:)-elm_learner.outRange(1,:));
%by = -(2./(elm_learner.outRange(2,:)-elm_learner.outRange(1,:))).*(elm_learner.outOffset + elm_learner.outRange(1,:)) - 1;
by = elm_learner.outOffset(1,:)+elm_learner.outRange(1, :);
str = sprintf('%s/Win.mat',datatarget);
funclocal_Matrix2TXT(elm_learner.wInp, str);
str = sprintf('%s/a.vec',datatarget);
funclocal_Matrix2TXT(elm_learner.a', str);
str = sprintf('%s/b.vec',datatarget);
funclocal_Matrix2TXT(elm_learner.b', str);
str = sprintf('%s/Wout.mat',datatarget);
funclocal_Matrix2TXT(elm_learner.wOut', str, true);
str = sprintf('%s/Ax.mat',datatarget);
funclocal_Matrix2TXT(Ax, str, true);
str = sprintf('%s/bx.vec',datatarget);
funclocal_Matrix2TXT(bx, str);
str = sprintf('%s/Ay.mat',datatarget);
funclocal_Matrix2TXT(Ay, str,true);
str = sprintf('%s/by.vec',datatarget);
funclocal_Matrix2TXT(by, str);

%Write matrix format
str = sprintf('%s/info',target);
fid = fopen(str,'wt');
fprintf(fid, 'elm+datascale');
fclose(fid);

end

