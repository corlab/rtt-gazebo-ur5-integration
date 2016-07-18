function [] = print_img(name, ext, trim, width, height)
%PRINT_IMG Summary of this function goes here
%   Detailed explanation goes here

if nargin < 1
    error(['no filename specified']);
    return
end

if nargin < 2
    ext = 'png';
end

if nargin < 3
    trim = 1;
end

if numel(name) < 5 || ~strcmpi(name(end-3:end), ['.' ext])
    name = [name '.' ext]; % Add the missing extension
end

if nargin < 5
    print(name, ['-d' ext]);
else
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 width/100 height/100]);
    print(name, ['-d' ext], '-r100');
end

if trim
    system(['convert ' name ' -trim ' name]);
end

end

