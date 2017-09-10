%This code calculated maximal muscle force for different muscle as a function
%of the motion range of the joints and the values of the routing matrix
%   Ali Marjaninejad September 2017
close all;clear all;clc;
r1=10; r2=7; r3=8; r4=12;
R=[-r1 -r1 r2 r2;
    -r3 r4 -r3 r4]/100; % in meters

q1_vec=0:1:90;
q2_vec=0:1:180;
j=0;
for q1=q1_vec
    for q2=q2_vec
        [th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2]=deal(q1,q1,q2,q2,.5,.8,.5);
        [ T_all_collect,T1,T2,T3,J_I_T ] = workspace_fcn(th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2); %% reading the Transform matrix from the BME504_workspace_fcn and also plotting the workspace
        close all;
        j=j+1;
        percent=100*(q1+1)/(length(q1_vec))

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
            if x(i)<=-0.5 || x(i)>=0.5
                Fmax(i)=0;
            else
                Fmax(i)=1-(x(i)/w)^2;
            end
        end
        Fmax_c(j,:)=Fmax(:);
        xx(j)=T_all_collect(1,4,:);
        yy(j)=T_all_collect(2,4,:);
    end
end

%%
q1t=q1_vec;
q2t=q2_vec;
for m=1:4
    subplot(2,2,m)
    X=reshape(xx(:),length(q2t),length(q1t));
    Y=reshape(yy(:),length(q2t),length(q1t));
    Fmax_reshaped=reshape(Fmax_c(:,m),length(q2t),length(q1t));
    mesh(X,Y,Fmax_reshaped)                                                   %% *pi/180 is to change it from degrees to radian
    title(['Muscle ',num2str(m)]);
    xlabel('x');ylabel('y');zlabel('F_m_a_x');
end