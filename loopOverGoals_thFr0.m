close all
clear all
global param

param.thFrisOrient=pi;

% Set goals for frisbee behavior (linear velocity, direction, and spin)
param.velGoal = 14;          % Desired maximum frisbee linear velocity
param.angGoal = pi/4;       % Desired frisbee direction at max. velocity
param.spinGoal = -50;         % Desired frisbee spin at max. velocity SHOULD BE NEGATIVE  

% Choose what error optimization will try to minimize - velocity, angle,
% spin, or some combination. Used by objFunc.m
param.objective = "V"; %velocity
% param.objective = "A";% angle
% param.objective = "S";% spin
% param.objective = "VA";
% param.objective = "VS";
% param.objective = "AS";
%  param.objective = "VAS";

DOF = [2,3];
Obj = ["V", "VA", "VS", "AS", "VAS", "A", "S",];
angGoal = [0 pi/4 pi/2 3*pi/4];
thFrisOrient = [0];

for fr = 1:length(thFrisOrient)
    for ang = 1:length(angGoal)
        for o = 1:length(Obj)
            for d = 1:length(DOF)
                param.dof= DOF(d);
                param.objective = Obj(o);
                param.angGoal = angGoal(ang);
                param.thFrisOrient = thFrisOrient(fr);
                tic
                OLsim_toLoop()
                toc
            end
        end
    end
end
