% GaitParameter1 returns the following variables for a given number of
% steps previously defined in the variable 'steps':
% LambdaX: Step length which varies depending of the step. For the
%          first and one before last steps it is half the normal steplength.
%          (1 x steps) Vector.
% LambdaY: Varying Step length. (1 x steps) ??? not sure what it is yet.
% Alpha_foot: (1 x steps) Vector containing rotation angles for each step.
% Nu: Walking speed which is always equal to 'velo' excpet for the first
%     and last steps.
% Lift: Height of ankle lift.
% LeftFootMoving: Is a vector indicating with 1 and 0 whether the left foot
%                 is in the air or not respectively.

function [LambdaX,LambdaY,Alpha_foot,Nu,Lift,Wait,LeftFootMoving]=GaitParameter1

global totalnumberofsteps hip_Comy
global step_width Tcycle Height steplength

steps=6;
% i find lift height 0.04 is a bit high for real icub walking


% velo=2*steplength/Tcycle;         % 0.22 -- ORIGINALE LINE
velo=steplength/Tcycle;                                                % 0.22;

%%%%%%%%%%%%
LambdaX(1)        =  0.5*steplength;                                       % step length 0.15 angle of attack is nearly 71.3885
LambdaY(1)        = -((-1)^1)*step_width*hip_Comy*2.0;                     % outside edge y position of footprint
Alpha_foot(1)     =  0;
Nu(1)             =  0.2*velo;                                             % walking speed
Lift(1)           =  1*Height;                                             % height of the lift
Wait(1)           =  0;                                                    % waiting moment after DS phase
LeftFootMoving(1) = (-(-1)^1+1)/2;                                         % 1 should mean swing leg

LambdaX(2)        =  0.5*steplength;                                       % step length 0.15 angle of attack is nearly 71.3885
LambdaY(2)        = -((-1)^2)*step_width*hip_Comy*2.0;                     % outside edge y position of footprint
Alpha_foot(2)     =  0;
Nu(2)             =  0.5*velo;                                             % walking speed
Lift(2)           =  1*Height;                                             % height of the lift
Wait(2)           =  0;                                                    % waiting moment after DS phase
LeftFootMoving(2) = (-(-1)^2+1)/2;                                         % 1 should mean swing leg

alpha_deg = 0; 
for i=3:steps-2
    LambdaX(i)=steplength;                                                 % step length 0.15 angle of attack is nearly 71.3885
    LambdaY(i)=-((-1)^i)*step_width*hip_Comy*2.0;                          % outside edge y position of footprint
    
    Alpha_foot(i) = alpha_deg*pi/180;                                      % 30*(rand(1)-0.5)*pi()/180; Generates angles between -15 and 15 degrees.
    
    Nu(i)=velo;                                                        % walking speed JORH: ORIGINALE WITHOUT 0.5
    Lift(i)=Height;                                                        % height of the lift
    Wait(i)=0;                                                             % waiting moment after DS phase
    LeftFootMoving(i)=(-(-1)^i+1)/2;                                       % 1 should mean swing leg
end
%
LambdaX(steps-1)=0.5*steplength;                                           % step length 0.15 angle of attack is nearly 71.3885
LambdaY(steps-1)=-((-1)^(steps-1))*step_width*hip_Comy*2.0;                % outside edge y position of footprint
Alpha_foot(steps-1)=0;
Nu(steps-1)=0.5*velo;                                                      % walking speed
Lift(steps-1)=1*Height;                                                    % height of the lift
Wait(steps-1)=0;
LeftFootMoving(steps-1)=(-(-1)^(steps-1)+1)/2;

LambdaX(steps)=0.0*steplength;                                             % step length 0.15 angle of attack is nearly 71.3885
LambdaY(steps)=-((-1)^steps)*1*hip_Comy*2.0;                               % outside edge y position of footprint
Alpha_foot(steps)=0;
Nu(steps)=0.2*velo;                                                        % walking speed
Lift(steps)=1*Height;                                                      % height of the lift
Wait(steps)=0;
LeftFootMoving(steps)=(-(-1)^steps+1)/2;

totalnumberofsteps = steps;