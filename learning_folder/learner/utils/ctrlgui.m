classdef ctrlgui < handle
    %CTRLGUI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dims = [];
        minmax = [];
        enable = [];
        values = [];
        f = [];
        ctrls = [];
    end
    
    methods
        function cg = ctrlgui(numDims, minmaxValues, parentup)
            if nargin < 1
                numDims = 3;
            end
            
            if nargin < 2
                minmaxValues = [-ones(numDims,1) ones(numDims,1)];
            end
            
            cg.dims = numDims;
            cg.minmax = minmaxValues;
            cg.enable = ones(cg.dims,1);
            cg.values = zeros(cg.dims,1);
            
            if nargin < 3
                cg.f = figure('Visible','off','Position',[400,300,500,50*cg.dims]);
                set(cg.f,'MenuBar','none');
                set(cg.f,'Units','normalized');
                % Initialize the GUI.
                % Assign the GUI a name to appear in the window title.
                set(cg.f,'Name','dimctrl');
                % Move the GUI to the center of the screen.
                movegui(cg.f,'center');
                % Make the GUI visible.
                set(cg.f,'Visible','on');
            end
            
            cg.ctrls = zeros(cg.dims,4);
            for d=1:cg.dims
                if nargin < 3
                    cg.ctrls(d,1) = uipanel('Title', ['dim' num2str(d)],...
                        'Units','normalized',...
                        'Position', [0 1-d/cg.dims 1 1/cg.dims],...
                        'FontSize', 12, 'BackgroundColor', 'white');
                else
                    cg.ctrls(d,1) = uipanel('Title', ['dim' num2str(d)],...
                        'Parent', parentup, 'Units','normalized',...
                        'Position', [0 1-d/cg.dims 1 1/cg.dims],...
                        'FontSize', 12, 'BackgroundColor', 'white');
                end
                cg.ctrls(d,2) = uicontrol('Parent', cg.ctrls(d,1),'Units','normalized',...
                    'Style','radiobutton','String','A/C',...
                    'Position', [0 .1 .1 1],'Value',1,...
                    'Callback',{@cg.dimctrl_Callback});
                cg.ctrls(d,3) = uicontrol('Parent', cg.ctrls(d,1),'Units','normalized',...
                    'Position', [.1 .1 .65 1],...
                    'Style','slider',...
                    'Min', cg.minmax(d,1), 'Max', cg.minmax(d,2),...
                    'Value', cg.minmax(d,1) + (cg.minmax(d,2)-cg.minmax(d,1))/2,...
                    'Callback',{@cg.dimctrl_Callback});
                cg.ctrls(d,4) = uicontrol('Parent', cg.ctrls(d,1),'Units','normalized',...
                    'Position', [.75 .1 .25 1],...
                    'String','0');
                
                %if (cg.minmax(d,1) > 0) || (cg.minmax(d,2) < 0)
                %    set(cg.ctrls(d,3),'Value',(cg.minmax(d,2)-cg.minmax(d,1))/2);
                %end
            end
        end
        
        function setValues(cg, values)
            for i=1:min(length(values), cg.dims)
                cg.values(i) = min(cg.minmax(i,2), max(values(i), cg.minmax(i,1)));
                set(cg.ctrls(i,3), 'Value', cg.values(i));
                set(cg.ctrls(i,4),'String',num2str(cg.values(i)));
            end
        end
        
        function setEnabled(cg, dims)
            for i=1:min(length(dims), cg.dims)
                set(cg.ctrls(i,2),'Value',dims(i));
                cg.enable(i) = dims(i);
            end
        end
        
        function dimctrl_Callback(cg,source,event)
            [dim typ] = find(cg.ctrls==source);
            if typ == 2
                cg.enable(dim) = get(source,'Value');
                %enable
                notify(cg,'EnableUpdate');
            elseif typ == 3
                cg.values(dim) = get(source,'Value');
                set(cg.ctrls(dim,4),'String',num2str(cg.values(dim)));
                %values
                notify(cg,'ValuesUpdate');
            end
        end
    end
    
    events
        ValuesUpdate
        EnableUpdate
    end
    
end

