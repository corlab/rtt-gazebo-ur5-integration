classdef SixDOFManipulator < handle
    %SixDOFMANIPULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        link1;
        link2;
        link3;
        link4;
        link5;
        link6;
        base;
        
        point1;
        point2;
        point3;
        point4;
        point5;
        point6;
        
    end
    
    methods
        function obj=SixDOFManipulator()
        obj.link1=[0.089159;0];
        obj.link2=[0.42500;0];
        obj.link3=[0.39225;0];
        obj.link4=[0.10915 + 0.1197 - 0.13585;0];
        obj.link5=[0.09465;0];
        obj.link6=[0.0823;0];
        obj.base=[0;0];
        
%{
Taken from https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_description/urdf/ur5.urdf.xacro
<property name="ur5_d1" value="0.089159" />
  <property name="ur5_a2" value="-0.42500" />
  <property name="ur5_a3" value="-0.39225" />
  <property name="ur5_d4" value="0.10915" />
  <property name="ur5_d5" value="0.09465" />
  <property name="ur5_d6" value="0.0823" />

  <!-- Arbitrary offsets for shoulder/elbow joints -->
  <property name="shoulder_offset" value="0.13585" />  <!-- measured from model -->
  <property name="elbow_offset" value="-0.1197" /> <!-- measured from model -->

  <!-- link lengths used in model -->
  <property name="shoulder_height" value="${ur5_d1}" />
  <property name="upper_arm_length" value="${-ur5_a2}" />
  <property name="forearm_length" value="${-ur5_a3}" />
  <property name="wrist_1_length" value="${ur5_d4 - elbow_offset - shoulder_offset}" />
  <property name="wrist_2_length" value="${ur5_d5}" />
  <property name="wrist_3_length" value="${ur5_d6}" />
%}

        obj.point1=obj.link1;
        obj.point2=obj.point1+obj.link2;  
        obj.point3=obj.point2+obj.link3;
        obj.point4=obj.point3+obj.link4;
        obj.point5=obj.point4+obj.link5;
        obj.point6=obj.point5+obj.link6;
        end
        
        function eepos=fkin(this, angle1, angle2)
            if nargin<7
                angle6=angle1(6,:);
                angle5=angle1(5,:);
                angle4=angle1(4,:);
                angle3=angle1(3,:);
                angle2=angle1(2,:);
                angle1=angle1(1,:);
            end
            eepos=zeros(2,size(angle1,2));
            for i=1:size(angle1,2)
                a1=angle1(1,i);
                a2=angle2(1,i);
                a3=angle3(1,i);
                a4=angle4(1,i);
                a5=angle5(1,i);
                a6=angle6(1,i);

                this.point1= this.rotMat(a1)*this.link1+this.base;
                this.point2= this.rotMat(a1)*((this.rotMat(a2)*this.link2) + this.link1)+this.base;
                this.point3= this.rotMat(a1)*((this.rotMat(a2)*(this.rotMat(a3)*this.link3 + this.link2)) + this.link1)+this.base;
                this.point4= this.rotMat(a1)*((this.rotMat(a2)*(this.rotMat(a3)*( this.rotMat(a4)*this.link4+ this.link3) + this.link2)) + this.link1)+this.base;
                this.point5= this.rotMat(a1)*((this.rotMat(a2)*(this.rotMat(a3)*( this.rotMat(a4)*(this.rotMat(a5)*this.link5 + this.link4)+ this.link3) + this.link2)) + this.link1)+this.base;
                this.point6= this.rotMat(a1)*((this.rotMat(a2)*(this.rotMat(a3)*( this.rotMat(a4)*(this.rotMat(a5)*(this.rotMat(a6)*this.link6 + this.link5) + this.link4)+ this.link3) + this.link2)) + this.link1)+this.base;


                eepos(:,i)=this.point6;
                
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

