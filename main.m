clc
clear
close all

addpath('lib')

% Select state space  
ss = stateSpaceSE2;

% Build state validator
sv = validatorOccupancyMap(ss);

load workspace\mapsStatic.mat

% Add map to state validator
sv.Map = map2D_1;

sv.ValidationDistance = 0.01;

ss.StateBounds = [sv.Map.XWorldLimits; sv.Map.YWorldLimits; [-pi pi]];

% Build planner using ss and sv
planner = planPSOQRRTStar(ss, sv);
planner.MaxConnectionDistance = 15;
planner.MaxIterations = 1e5;

% Continue planning after reaching goal 
planner.ContinueAfterGoalReached = true;

% Time limit for planning
% planner.MaxTime = 10;

% for map2D_1
start = [40, 40, 0];
goal = [70, 40, 0];

% for map2D_2
% start = [30, 45, 0];
% goal = [80, 45, 0];

% for map2D_3
% start = [20, 10, 0];
% goal = [80, 90, 0];

% for map2D_4
% start = [20, 90, 0];
% goal = [80, 80, 0];

% for mapMaze_1
% start = [30, 70, 0];
% goal = [60, 30, 0];

% for mapMaze_2
% start = [10, 10, 0];
% goal = [90, 90, 0];

% for mapMaze_3
% start = [60, 60, 0];
% goal = [10, 70, 0];

% for mapMaze_4
% start = [10, 10, 0];
% goal = [10, 110, 0];

% Planning step
[pthObj, solnInfo] = plan(planner, start, goal);

% 2D Plot of generated nodes and final path
f1 = figure;
f1.Position = [400 200 600 500];
hold on
show(sv.Map)
plot(solnInfo.TreeData(:, 1), solnInfo.TreeData(:, 2), '.-')
plot(pthObj.States(:, 1), pthObj.States(:, 2), 'r-', 'LineWidth', 2)
plot(start(1), start(2), 'ko')
plot(goal(1), goal(2), 'ko')
title("PSO Q-RRT* | Map 2D-1")