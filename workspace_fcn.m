%This code shows the workspace (all the points that can be reached) for a
%2DOF limb with the first and second joint angles moving from th1_min to 
%th1_max and th2_min and th2_max respectively. The lenght of the first and 
%second link is d1 and d2 respectively.
%e.g.:[th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2]=deal(0,90,0,180,1,10,10);
%   Ali Marjaninejad September 2017
function [ T_all_collect,T1,T2,T3,J_I_T] = workspace_fcn(th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2)
%clear all;close all;clc
% precision_in_degrees=0.4;
% %datapoints=90*5;
% th1_min=0;th1_max=90;
% th2_min=0;th2_max=180;
% d1=2;d2=1;
datapoints=max(th1_max-th1_min,th2_max-th2_min)/precision_in_degrees;
if datapoints==0
    datapoints=1;
end
th1=linspace(th1_min,th1_max,datapoints);
th2=linspace(th2_min,th2_max,datapoints);
%% mini functions
CT_z=@(th,d) [cosd(th) -sind(th) 0 d;
         sind(th) cosd(th) 0 0;
         0 0 1 0;
         0 0 0 1];
J_T=@(d1_f,d2_f,th1_f,th2_f) [-d2_f*sind(th1_f+th2_f)-d1_f*sind(th1_f) d2_f*cosd(th1_f+th2_f)+d1_f*cosd(th1_f);
    -d2_f*sind(th1_f+th2_f) d2_f*cosd(th1_f+th2_f)];    % Jacobian Transpose 
%% calculating workspace (all, not only boundries)
i=0;
for t_th1=th1
    for t_th2=th2
        T1=CT_z(t_th1,0);
        T2=CT_z(t_th2,d1);
        T3=CT_z(0,d2);
        T_all=T1*T2*T3;
        i=i+1;
        T_all_collect(:,:,i)=T_all;
    end
end
%% acobian inverse transpose
J_I_T=pinv(J_T(d1,d2,th1,th2));
%% Visualization
% x=T_all_collect(1,4,:);
% y=T_all_collect(2,4,:);
% plot(x(:),y(:),'r*')
% xlabel('x');ylabel('y');title('Workspace')
end

