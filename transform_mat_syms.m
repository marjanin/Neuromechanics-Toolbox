%This code calculates the symbolic representation of the total transform 
%matrix for a multi-DOF joint (in the 2D space)with joint angles defined 
%in vector q and link lenghts defined in vector l
%   Eqs 2.6-2.14 in Valero-Cuevas 2015
%   Ali Marjaninejad September 2016
clear all;close all;clc;
%% define symbolics here
syms q1 q2 l1 l2
%% Costum made mini function here
CT_z=@(th,d) [cos(th) -sin(th) 0 d;
         sin(th) cos(th) 0 0;
         0 0 1 0;
         0 0 0 1];
%% Defining transformation matrixes here
T0_1=CT_z(q1,0)
T1_2=CT_z(q2,l1)
T2_3=CT_z(0,l2)
%% Calculating the final results here
T0_3=T0_1*T1_2*T2_3
simplified_T0_3=simplify(T0_3)
J=jacobian([T0_3(1:2,4);q1+q2],[q1 q2]);
simplified_J=simplify(J)
%% Solve for real numbers here
q1=0;q2=0;l1=5;l2=10;
T_numbers=subs(T0_3)
%% End of the code






%% Costum made mini function here 2
% CT=symfun([cos(q1) -sin(q1) 0 l1;
%          sin(q1) cos(q1) 0 0;
%          0 0 1 0;
%          0 0 0 1],[q1,l1])