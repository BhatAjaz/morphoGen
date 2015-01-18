% The transformations of the kinematics-chains for the industrial platform:
% P' = P * T_position * T1 * R1 * J1 * T2 * R2 * J2 * ... * T6 * R6 * J6
%   
% If P ist the origin of the world coordinate system, P' will be the point at the flange of the robot. 
% Jx refers to rotation matrices which describe rotation about the Z-Axis (e.g. J6 for the rotation about the Z-Axis by the 6th joint angle)
% Rx refers to rotation matrices about the X-Axis (e.g. R5)
% Tx refers to translation matrices (e.g. T5)
 
clear

% generating the translation matrices tx

t = [1 0 0;0 1 0; 0 0 1; 0 0 0];

% % % % RX130 Position as determined by calibration procedure - old Calibration
% % % % t0 = [  0.99986299     0.015008902  -0.0069812603   -427.30289;
% % % %        -0.014970048    0.99987235    0.0055848885   -781.08851;
% % % %         0.0070641922  -0.0054796135  0.99996003     -306.98309;
% % % %         0              0             0               1];
    
    
%     %RX130 Position as determined by calibration procedure - Calibration
%     %06122013
%   t0= [ -0.923487086537064    0.382450378735583   -0.0300550961451632       818.1;
%         -0.315511382693663   -0.801742600590122   -0.507603555729982        880.1;
%         -0.218229623080869   -0.459282603855184    0.861066386178199        309.59;
%          0                    0                    0                        1.0000];
   

  t0 = [0.9999   -0.0129    0.0056         722.67;
        0.0129    0.9999    0.0006         902.4;
       -0.0056   -0.0005    1.0000         309.84;
        0         0         0              1.0000];
         
         
T1=[0;0;330;1];
T2=[0;171;220;1];
T3=[0;-625;0;1];
T4=[0;-132;-171;1];
T5=[0;0;493;1];
T6=[0;-110;0;1];
T7=[0;0;220;1];

t1 = [t T1]
t2 = [t T2]
t3 = [t T3]
t4 = [t T4]
t5 = [t T5]
t6 = [t T6]
t7 = [t T7]

% generating the rotation matrices rx around X axes

R1=[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0);0 0 0];
R2=[1 0 0; 0 cos(-1.5708) -sin(-1.5708); 0 sin(-1.5708) cos(-1.5708);0 0 0];
R3=[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0);0 0 0];
R4=[1 0 0; 0 cos(1.5708) -sin(1.5708); 0 sin(1.5708) cos(1.5708);0 0 0];
R5=[1 0 0; 0 cos(-1.5708) -sin(-1.5708); 0 sin(-1.5708) cos(-1.5708);0 0 0];
R6=[1 0 0; 0 cos(1.5708) -sin(1.5708); 0 sin(1.5708) cos(1.5708);0 0 0];

r =[0; 0; 0; 1];
r1 = [R1 r]
r2 = [R2 r]
r3 = [R3 r]
r4 = [R4 r]
r5 = [R5 r]
r6 = [R6 r]

% generating the rotation matrices jx around z axes
%syms u1 u2 u3 u4 u5 u6;
% u1 =  30 / 180 * pi;
% u2 = -45 / 180 * pi;
% u3 =  30 / 180 * pi;
% u4 =  90 / 180 * pi;
% u5 =  20 / 180 * pi;
% u6 = -20 / 180 * pi;

u1 =0.698;
u2 =0.723;
u3 =1.39;
u4 = -1.89;
u5 = -0.63;
u6 = 0;



J1=[cos(u1) -sin(u1) 0; sin(u1) cos(u1) 0; 0 0 1; 0 0 0];
J2=[cos(u2) -sin(u2) 0; sin(u2) cos(u2) 0; 0 0 1; 0 0 0];
J3=[cos(u3) -sin(u3) 0; sin(u3) cos(u3) 0; 0 0 1; 0 0 0];
J4=[cos(u4) -sin(u4) 0; sin(u4) cos(u4) 0; 0 0 1; 0 0 0];
J5=[cos(u5) -sin(u5) 0; sin(u5) cos(u5) 0; 0 0 1; 0 0 0];
J6=[cos(u6) -sin(u6) 0; sin(u6) cos(u6) 0; 0 0 1; 0 0 0];

j1 = [J1 r]
j2 = [J2 r]
j3 = [J3 r]
j4 = [J4 r]
j5 = [J5 r]
j6 = [J6 r]

p = [0; 0; 0; 1];

% first correct solution
% m1 = t0 * t1 * r1 * j1 * t2 * r2 * j2 * t3 * r3 * j3 * t4 * r4 * j4 * t5 * r5 * j5 * t6 * r6 * j6 * t7;
% m1
% m1 * p

% first correct solution
 m1 = (inv(t0)) * t1 * r1 * j1 * t2 * r2 * j2 * t3 * r3 * j3 * t4 * r4 * j4 * t5 * r5 * j5 * t6 * r6 * j6 * t7;
vpa(m1 * p)


% second correct solution (equivalent to first solution)
% m2 = (t7' * j6' * r6' * t6' * j5' * r5' * t5' * j4' * r4' * t4' * j3' * r3' * t3' * j2' * r2' * t2' * j1' * r1' * t1' * t0');
% m2
% (p' * m2)'
