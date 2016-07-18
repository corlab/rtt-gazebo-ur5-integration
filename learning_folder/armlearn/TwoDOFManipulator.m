classdef TwoDOFManipulator < handle
    %TWODOFMANIPULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        link1;
        link2;
        base;
        
        point1;
        point2;
        
    end
    
    methods
        function obj=TwoDOFManipulator()
        obj.link1=[0.5;0];
        obj.link2=[0.5;0];
        obj.base=[0;0];
        
        obj.point1=obj.link1;
        obj.point2=obj.point1+obj.link2;  
        end
        
        function eepos=fkin(this, angle1, angle2)
            if nargin<3
                angle2=angle1(2,:);
                angle1=angle1(1,:);
            end
            eepos=zeros(2,size(angle1,2));
            for i=1:size(angle1,2)
                a1=angle1(1,i);
                a2=angle2(1,i);
                
                this.point1= this.rotMat(a1)*this.link1+this.base;
                this.point2= this.rotMat(a1)*((this.rotMat(a2)*this.link2) + this.link1)+this.base;
                eepos(:,i)=this.point2;
                
            end
        end
        
        function plotState(this, h)
            if nargin<2
                h=gcf();
            else
                figure(h);
            end
            
            hold on;
            
            plot([this.base(1), this.point1(1), this.point2(1)],[this.base(2), this.point1(2), this.point2(2)]);
    
            
        end
        
        function m=rotMat(this, angle)
            m=[cos(angle), -sin(angle); sin(angle), cos(angle)];
        end
        
        
        
    end
    
end

