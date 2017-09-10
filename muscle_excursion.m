%This code calculated muscle excursion for different muscle as a function
%of the motion range of the joints and the values of the routing matrix
%   Ali Marjaninejad September 2017
close all;clear all;clc;
[th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2]=deal(0,90,0,180,.5,20,20);
[ T_all_collect ] = workspace_fcn(th1_min,th1_max,th2_min,th2_max,precision_in_degrees,d1,d2); %% reading the Transform matrix from the BME504_workspace_fcn and also plotting the workspace
x=T_all_collect(1,4,:);
y=T_all_collect(2,4,:);
th1_0=0;th2_0=0;
datapoints=max(th1_max-th1_min,th2_max-th2_min)/precision_in_degrees;
th1=linspace(th1_min,th1_max,datapoints);
th2=linspace(th2_min,th2_max,datapoints);
r1=10;r2=7;r3=8;r4=12;
R=[-r1 -r1 r2 r2;-r3 r4 -r3 r4];
ds2=zeros(datapoints^2,4);
for m=1:4
    i=0;
    for t_th1=th1
        for t_th2=th2
            i=i+1;            R_Transp=R';
            ds2(i,m)=-R_Transp(m,:)*[t_th1-th1_0;t_th2-th2_0];
        end
    end
end
figure
for m=1:4
    subplot(2,2,m)
    X=reshape(x(:),length(th1),length(th1));
    Y=reshape(y(:),length(th1),length(th1));
    DS2=reshape(ds2(:,m),length(th1),length(th1));
    mesh(X,Y,DS2*pi/180)                                                   %% *pi/180 is to change it from degrees to radian
    title(['Muscle ',num2str(m)]);
    xlabel('x');ylabel('y');zlabel('ds');
end