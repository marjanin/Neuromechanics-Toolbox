function [ T_total ] = transform_mat( q,l )
%This code calculates the total transform matrix for a multi-DOF joint 
%(in the 2D space)with joint angles defined in vector q and link lenghts 
%defined in vector l.
%   Eqs 2.6-2.14 in Valero-Cuevas 2015
%   Ali Marjaninejad September 2017

%% initializing and redefining inputs of the transformation matrix 
q_length=length(q);
l_length=length(l);
q_r=reshape(q,[1,q_length]);        % making sure that q is a row vector
l_r=reshape(l,[1,l_length]);        % making sure that l is a row vector
q_ready=[q_r 0];                      % making q vector ready to be used in transformation matrices
l_ready=[0 l_r];                      % making l vector ready to be used in transformation matrices
%% default transform matrix
CT_z=@(th,d) [cosd(th) -sind(th) 0 d;
         sind(th) cosd(th) 0 0;
         0 0 1 0;
         0 0 0 1];
%%
T_total=eye(4);
for loop_c1=1:q_length+1
    T{loop_c1}=CT_z(q_ready(loop_c1),l_ready(loop_c1));
    T_total=T_total*T{loop_c1};
end

