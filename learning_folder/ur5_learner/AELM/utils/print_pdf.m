%PRINT_PDF  Prints figures to pdfs better than MATLAB does
%
% Examples:
%   print_pdf filename
%   print_pdf(filename, fig_handle)
%   print_pdf(name, fig, eps, bw, zbuffer)
%
% This function saves a figure as a pdf nicely, without the need to specify
% multiple options. It improves on MATLAB's print command (using default
% options) in several ways:
%   - The figure borders are cropped
%   - Fonts are embedded (as subsets)
%   - Lossless compression is used on vector graphics
%   - High quality jpeg compression is used on bitmaps
%
% This function requires that you have ghostscript installed on your system
% and that the executable binary is on your system's path.
%
%IN:
%   filename - string containing the name (not path) of the file the figure
%              is to be saved as. A ".pdf" extension is added if not there
%              already. The figure is saved in the current directory.
%   fig_handle - The handle of the figure to be saved. Default: current
%                figure.
%
% Copyright (C) Oliver Woodford 2008

% This function is inspired by Peder Axensten's SAVEFIG (fex id: 10889)
% which is itself inspired by EPS2PDF (fex id: 5782)

% $Id: print_pdf.m,v 1.9 2008/11/08 00:36:06 ojw Exp $

function print_pdf(name, fig, eps, bw, zbuffer)
if nargin < 2
    fig = gcf;
end
if nargin < 3
    eps = false;
end
if nargin < 4
    bw = false;
end
if nargin < 5
    zbuffer = false;
end
% Get current image dimensions
set(fig, 'Units', 'Inches');
sz = get(fig, 'Position');
sz(1:2) = 0;
% Set paper units
set(fig, 'PaperUnits', 'Inches', 'PaperSize', sz(3:4), 'PaperPosition', sz);

% Construct the filename
if numel(name) < 5 || ~strcmpi(name(end-3:end), '.pdf')
    name = [name '.pdf']; % Add the missing extension
end
seps = strfind(name, filesep);
if size(seps,2) == 0
    name = [cd filesep name]; % Add the path to the current directory
else
    if seps(1) ~= 1
        name = [cd filesep name]; % Add the path to the current directory
    end
end

% Print to eps file
%tmp_nam = [tempname '.eps'];
tmp_nam = [name(1:end-3) 'eps'];
device = '-depsc2';
if bw
    device = '-deps2';
end
render = '-painters';
cropping = '';
if zbuffer
    render = '-zbuffer';
    %render = '-opengl';
    cropping = '-loose';
end

if isnumeric(fig)
    wndnr=fig;
else
    wndnr=fig.Number;
end
print(device, '-noui', render, cropping, ['-f' num2str(wndnr)], '-r600', tmp_nam);

% Construct the command string for ghostscript. This assumes that the
% ghostscript binary is on your path - you can also give the complete path.
cmd = 'gs';
if ispc
    cmd = [cmd 'win32c.exe'];
end
cmd = [cmd ' -q -dNOPAUSE -dBATCH -dEPSCrop -sDEVICE=pdfwrite -dPDFSETTINGS=/prepress -sOutputFile="' name '" -f "' tmp_nam '"'];
% Convert to pdf
system(cmd);

system(['pdfcrop ' name ' ' name]);

% Delete the temporary file
if ~eps
    delete(tmp_nam);
end
