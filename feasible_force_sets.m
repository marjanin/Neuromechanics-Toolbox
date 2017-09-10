%This code calculated feasible torque spaces and feasible wrench spaces as 
%a function of the motion range of the joints and the values of the routing
%matrix
%   Ali Marjaninejad September 2017
close all;clear all;clc
% inputs
dim=4

j=1

r1=10; r2=7; r3=8; r4=12;
R=[-r1 -r1 r2 r2;
    -r3 r4 -r3 r4]/100; % in meters
q1_vec=180*[   1.1908    1.0621    0.8134    0.7008    0.6014    0.5182    0.4536    0.4092    0.3867    0.3892]/pi;
q2_vec=180*[   1.8364    1.8755    1.8755    1.8364    1.7721    1.6835    1.5708    1.4329    1.2661    1.0616]/pi;
for iii=1:10
figure
q1=q1_vec(iii);q2=q2_vec(iii);

%Transformation matrices and Jacobian
[th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2]=deal(q1,q1,q2,q2,.5,.8,.5);
[ T_all_collect,T1,T2,T3,J_I_T ] = workspace_fcn(th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2); %% reading the Transform matrix from the BME504_workspace_fcn and also plotting the workspace

% alpha
[vertices,count] = ncube(dim);
%F0
        %% muscle length calculation
        dq1=q1-45;
        dq2=q2-90;
        oml=[20;10;20;15];
        dml=(-R')*[dq1;dq2]; %d lenght muscle 1
        ml=oml+dml;
        x=(ml./oml)-1;
        Fmax=zeros(4,1);
        w=.5;
        for i=1:4
            if x(i)<=-0.5 || x(i)>=0.5;
                Fmax(i)=0;
            else
                Fmax(i)=1-(x(i)/w)^2;
            end
        end
        Fmax_c(j,:)=Fmax(:);
        xx(j)=T_all_collect(1,4,:);
        yy(j)=T_all_collect(2,4,:);
F0=35*diag([10*Fmax(1) 20*Fmax(1) 15*Fmax(1) 25*Fmax(1)]);
%%
%R_F0
Torq=R*F0;
%H
H=J_I_T*Torq;

%F=vertices*F0;

%a plot
% subplot(221)
% hold on
% cubeindexes=[1 2 6 5 1 3 7 5 1 2 4 8 6 8 7 3 4];;
% plot3(vertices(cubeindexes,1),vertices(cubeindexes,2),vertices(cubeindexes,3))

%F plot
% subplot(222)
% hold on
% plot3(F(cubeindexes,1),F(cubeindexes,2),F(cubeindexes,3))
% 

        Torq_total=[];
    for i=1:count
        Torq_total= [Torq_total;(Torq*vertices(i,:)')'];
    end
        CoHui_R_F_total = convhull(Torq_total); %Convex Hull indexes
            subplot(211)
            hold on
            plot(Torq_total(CoHui_R_F_total,1),Torq_total(CoHui_R_F_total,2),'r-')
            
        H_total=[];
    for i=1:count
        H_total= [H_total;(H*vertices(i,:)')'];
    end
        CoHui_H_total = convhull(H_total); %Convex Hull indexes
            subplot(212)
            hold on
            plot(H_total(CoHui_H_total,1),H_total(CoHui_H_total,2),'r-')
           
markercolors=[
[1 1 0];
[1 0 1];
[0 1 1];
[1 0 0];
[0 1 0];
[0 0 1];
[1 .4 0];
[0 0 0];
[1 1 0];
[1 0 1];
[0 1 1];
[1 0 0];
[0 1 0];
[0 0 1];
[1 .4 0];
[0 0 0]];
    for i=1:count
%         subplot(221)
%         plot3(vertices(i,1),vertices(i,2),vertices(i,3),'*','color',(markercolors(i,:)));
%         subplot(222)
%         plot3(F(i,1),F(i,2),F(i,3),'*','color',(markercolors(i,:)));
        subplot(211)
        plot(Torq_total(i,1),Torq_total(i,2),'*','color',(markercolors(i,:)));
        subplot(212)
        plot(H_total(i,1),H_total(i,2),'*','color',(markercolors(i,:)));
        
    end
%     subplot(221);view(-30,25);xlabel('x');ylabel('y');zlabel('z');title('Activation space (a)')
%     subplot(222);view(-30,25);xlabel('x');ylabel('y');zlabel('z');title('Muscle Force space (F)')
    subplot(211);xlabel('x');ylabel('y');title('Torqu space (T)')
    subplot(212);xlabel('x');ylabel('y');title('Wrench space (W)')
            
end          
            
            
            
            
            
            
            
            
%             %%
%      format compact 
%     h(1) = axes('Position',[0.2 0.2 0.6 0.6]);
%     vert = vertices;
%     fac = [1 2 3 4; ...
%         2 6 7 3; ...
%         4 3 7 8; ...
%         1 5 8 4; ...
%         1 2 6 5; ...
%         5 6 7 8];
%     patch('Faces',fac,'Vertices',vert,'FaceColor','r');  % patch function
%     material shiny;
%     alpha('color');
%     alphamap('rampdown');
%     view(30,30);