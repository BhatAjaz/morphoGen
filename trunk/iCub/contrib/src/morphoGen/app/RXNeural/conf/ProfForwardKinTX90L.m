
clear

% generating the translation matrices tx

t = [1 0 0;0 1 0; 0 0 1; 0 0 0];
 
t0 = [0.9999  0.0068 0.0099   723.06;  
     -0.0068  1.000  0.0019  -683.48; 
     -0.0099 -0.0020 0.9999   310.0;
      0.0     0.0    0.0      1.0]; % Calibration 1012
      
T1=[0;0;252;1];
T2=[50;161;226;1];
T3=[0;-500;0;1];
T4=[0;-157;-111;1];
T5=[0;0;393;1];
T6=[0;-100;0;1];
T7=[0;0;220;1]

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
% syms u1 u2 u3 u4 u5 u6;
u1 = -46.21 / 180 * pi;
u2 =  52.52 / 180 * pi;
u3 = 42.93 / 180 * pi;
u4 =  0 / 180 * pi;
u5 =  84.54 / 180 * pi;
u6 =   -1.21 / 180 * pi;

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
m1 = (inv(t0)) * t1 * r1 * j1 * t2 * r2 * j2 * t3 * r3 * j3 * t4 * r4 * j4 * t5 * r5 * j5 * t6 * r6 * j6 * t7;
vpa(m1 * p)

% % second correct solution (equivalent to first solution)
% m2 = (t7' * j6' * r6' * t6' * j5' * r5' * t5' * j4' * r4' * t4' * j3' * r3' * t3' * j2' * r2' * t2' * j1' * r1' * t1' * t0');
% m2
% vpa((p' * m2)', 4)


