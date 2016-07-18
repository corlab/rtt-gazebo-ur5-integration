%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Copyright (c) 2013 R. F. Reinhart, CoR-Lab                 %%%
%%%          Bielefeld University, Germany, http://cor-lab.de           %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [h] = plotNDOFArm(q, c, clinks)

%robot configuration
n = size(q,2);
l = 1/n;
x0 = [0 0];

%colors and sizes
if nargin < 2
    c = 'k';
end
if nargin < 3
    clinks = [1 1 1];
end
jointRadius = 0.025;
sz = 0.05;

%for many joints
jointRadius = 0.02;
sz = 0.03;

h = gcf();
hold on; box on; grid on;
%xlim([-1.2 1.2]);
%ylim([-1.2 1.2]);
xlim([-0.75 1.1]);
ylim([-1.1 1.1]);
daspect([1 1 1]);
xlabel('x [m]');
ylabel('y [m]');

%draw base
br = sz + 0.75 * jointRadius;
rectangle('Position',[-br,-br,2*br,2*br],'Curvature',[1,1],'LineWidth',1,'LineStyle','-','EdgeColor',c);

%draw links
for i=1:n
    x1 = fkineNDOFArm(q(1:i), l);
    plotArmLink(sum(q(1:i)), l, x0', sz, c, clinks);
    x0 = x1;
end

%draw EE
draw_circle(x1(1), x1(2), jointRadius, c);

