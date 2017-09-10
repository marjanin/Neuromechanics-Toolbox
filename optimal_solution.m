%This code finds the optimal solution for in muscle activation space so
%that the output force becomes maximum/minimum (default:maximum) in the 
%desired direction (here Y axis).
%This code also plots the limb in 2D and shows the activation for each
%muscle as a bar plot.
%   Ali Marjaninejad September 2017
close all;clear all;clc;
r1=10; r2=7; r3=8; r4=12;
R=[-r1 -r1 r2 r2;
    -r3 r4 -r3 r4]/100; % in meters

q1=45;q2=90;

j=0;

q1_vec=180*[   1.1908    1.0621    0.8134    0.7008    0.6014    0.5182    0.4536    0.4092    0.3867    0.3892]/pi;
q2_vec=180*[   1.8364    1.8755    1.8755    1.8364    1.7721    1.6835    1.5708    1.4329    1.2661    1.0616]/pi;
for k=1:length(q1_vec)
q1=q1_vec(k);q2=q2_vec(k);
    
    %for q1=0:5:90
%     for q2=0:5:180
        [th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2]=deal(q1,q1,q2,q2,.5,.8,.5);
        [ T_all_collect,T1,T2,T3,J_I_T ] = workspace_fcn(th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2); %% reading the Transform matrix from the BME504_workspace_fcn and also plotting the workspace
        figure
        %close all;
        j=k;
         %j=j+1;
%         percent=100*j/(91*181/25)
        T1_2=T1*T2;
        T1_3=T1_2*T3;
        x0=0;y0=0;
        x1=T1_2(1,4);y1=T1_2(2,4);
        x2=T1_3(1,4);y2=T1_3(2,4);
        line([x0 x1],[y0 y1],'linewidth',8)
        hold on
        line([x1 x2],[y1 y2],'linewidth',8)
        plot(x0,y0,'ks','linewidth',20)
        plot(x1,y1,'r*','linewidth',25)
        plot(x2,y2,'y>','linewidth',20)
        xlabel('x');ylabel('y');title('2D limb')
        axis equal

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
%     end
% end

%%
F=35*diag([10*Fmax(1) 20*Fmax(1) 15*Fmax(1) 25*Fmax(1)])
a=[1 0 0 0]';
w=J_I_T*R*F*a


c_T=J_I_T*R*F;
A=[eye(4) ;-eye(4); c_T(1,:); -c_T(1,:)];
b=[ones(4,1);zeros(4,1); 0.001; -0.001];

X(:,k) = linprog(-c_T(2,:),A,b)

a=X(:,k);
Ft=c_T*a;
F_y(k)=Ft(2)
F_x(k)=Ft(1)
end


for ii=1:length(q1_vec)
    figure;
    bar(X(:,ii))
    xlabel('Muscle')
    ylabel('Activation')
    title(['Posture ',num2str(ii)])
end