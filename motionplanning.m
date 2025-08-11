%% Motion Planning MATLAB Simulation
% Author: Alfin Junaedy
% Affiliation: Masuta Lab - Toyama Prefectural University
% Date: 2025 - 08 - 08
% Citation:
%   If you use this code in your research, please cite:
%   [Your citation here]
% License:
%   This code is released under the MIT License.
%   See LICENSE file in the repository for details.

close all; clear all; clc;

% *********** Settings ***********
map = 2;                % Select map 1 or 2
start = [0.75 0.75];    % Starting position
goal = [9 1];           % Goal position
% --------------------------------
obsStep = 0.075;        % Obstacle movement step
obsDist = 0.75;         % Dynamic obstacle movement limit
safeDistOA = 0.80;      % Safety distance for OA
safeDistPP = 0.25;      % Safety distance for PP
pathStep = 0.25;        % Path planning step
timeoutPP = 3;          % Path planning timeout
pathUptErr = 0.15;      % Maximum displacement to update path
robotRds = 0.11;        % Robot radius
mpStep = 0.1;           % Motion planning step
goalRds = 0.2;          % Goal radius to reach
% ********************************

if (map == 1)
    load('wallnodes.mat');
    load('obsnodes.mat');
else
    load('wallnodes2.mat');
    load('obsnodes2.mat');
end

fig = figure; 
set(fig, 'WindowState', 'maximized'); 
hold on;
plot(start(2),start(1),'b^',goal(2),goal(1),'r^');
for i = 1:length(wallnodes)
    plot(wallnodes{i}(:,2),wallnodes{i}(:,1),'k-');
end
ylim([-1 11]); xlim([-1 11]);
ylabel('x [m]'); xlabel('y [m]'); axis equal; grid minor;
title("Motion Planning Simulation");
drawnow;
pltObs = []; pltPath = []; pltRobot = []; pltObs2 = []; path = []; nPath = 0;
tRobot = linspace(0,2*pi,15)';
cRobot0 = [robotRds*cos(tRobot) robotRds*sin(tRobot)];
dirOA = 0; dirPP = 0; dirMP = 0; weightOA = 0; gErr = 0; nObspos = 0;
curpos = start; curpos0 = curpos; goal0 = curpos; 

while 1
    % Dynamic obstacles ---------------------------------------------------
    obsnodes = updateobstacle_mex(obsStep,obsDist,wallnodes,obsnodes);
    %obsnodes = updateobstacle(obsStep,obsDist,wallnodes,obsnodes);
    try if (~isempty(pltObs)) set(pltObs,'xData',NaN,'yData',NaN); end 
    catch if (~isempty(pltObs)) delete(pltObs); end
    end
    pltObs = zeros(length(obsnodes),1);
    for i = 1:length(obsnodes)
        pltObs(i) = plot(obsnodes{i}(:,2),obsnodes{i}(:,1),'r-');
    end

    % Path planning -------------------------------------------------------
    if (nPath == 0)
        [path, nPath] = pathplanning_mex(curpos,goal,wallnodes,obsnodes,safeDistPP,pathStep,timeoutPP*0.3);
        %[path, nPath] = pathplanning(curpos,goal,wallnodes,obsnodes,safeDistPP,pathStep,timeoutPP);
        if (nPath > 0)
            if (path(1,1) == 0 && path(1,2) == 0) path(1,:) = goal; end
            dirPP = atan2d(path(1,2)-curpos(2),path(1,1)-curpos(1));
            try if (~isempty(pltPath)) set(pltPath,'xData',NaN,'yData',NaN); end 
            catch if (~isempty(pltPath)) delete(pltPath); end
            end
            pltPath = plot([curpos(2); path(1:nPath,2)],[curpos(1); path(1:nPath,1)],'b-');
        end
    else
        if (path(1,1) == 0 && path(1,2) == 0) path(1,:) = goal; end
        dirPP = atan2d(path(1,2)-curpos(2),path(1,1)-curpos(1));
    end

    % Obstacle avoidance --------------------------------------------------
    if (nPath > 0)
        [dirOA,weightOA,obspos,nObspos] = obstacleavoidance_mex(curpos,path(1,:),wallnodes,obsnodes,safeDistOA);
        %[dirOA,weightOA,obspos,nObspos] = obstacleavoidance(curpos,path(1,:),wallnodes,obsnodes,safeDistOA);
    else
        [dirOA,weightOA,obspos,nObspos] = obstacleavoidance_mex(curpos,goal,wallnodes,obsnodes,safeDistOA);
        %[dirOA,weightOA,obspos,nObspos] = obstacleavoidance(curpos,goal,wallnodes,obsnodes,safeDistOA);
    end

    % MOO -----------------------------------------------------------------
    O = [mpStep*cosd(dirOA) mpStep*sind(dirOA)];
    G = [mpStep*cosd(dirPP) mpStep*sind(dirPP)];
    D = weightOA*O + (1-weightOA)*G;
    dirMP = atan2d(D(2),D(1));
    curpos = curpos+[mpStep*cosd(dirMP) mpStep*sind(dirMP)];
    if (nPath > 0)
        gErr = sqrt(((curpos0(1)-path(1,1))*(curpos0(1)-path(1,1))) + ((curpos0(2)-path(1,2))*(curpos0(2)-path(1,2))));
        if (gErr <= goalRds && length(path(:,1)) > 1)
            goal0 = path(1,:);
            nPath = 0;
        else
            dPath = sqrt(((goal0(1)-path(1,1))*(goal0(1)-path(1,1))) + ((goal0(2)-path(1,2))*(goal0(2)-path(1,2))));
            rMin = 9999;
            for i = (pathUptErr*0.25)/dPath:(pathUptErr*0.25)/dPath:1
                o = path(1,:) + i * (goal0-path(1,:));
                r = sqrt(((o(1)-curpos0(1))*(o(1)-curpos0(1))) + ((o(2)-curpos0(2))*(o(2)-curpos0(2))));
                if (r < rMin) rMin = r; end
            end
            if (rMin > pathUptErr)
                nPath = 0;
                goal0 = curpos0;
            end
        end        
    end
    
    % Position update -----------------------------------------------------
    cRobot = cRobot0 + curpos0;
    try if (~isempty(pltRobot)) set(pltRobot,'xData',NaN,'yData',NaN); end 
    catch if (~isempty(pltRobot)) delete(pltRobot); end
    end
    pltRobot = plot(cRobot(:,2),cRobot(:,1),'k-');
    plot(curpos0(2),curpos0(1),'k.');
    lng = 0.35; VO = [curpos0(1)+lng*cosd(dirOA) curpos0(2)+lng*sind(dirOA)];
    lng = 0.4; VG = [curpos0(1)+lng*cosd(dirPP) curpos0(2)+lng*sind(dirPP)];
    lng = 0.3; VD = [curpos0(1)+lng*cosd(dirMP) curpos0(2)+lng*sind(dirMP)];
    try if (~isempty(pltObs2)) set(pltObs2,'xData',NaN,'yData',NaN); end 
    catch if (~isempty(pltObs2)) delete(pltObs2); end
    end
    pltObs2 = zeros(100,1);
    pltObs2(1) = plot([curpos0(2) VG(2)],[curpos0(1) VG(1)],'b-','LineWidth',5);
    pltObs2(2) = plot([curpos0(2) VO(2)],[curpos0(1) VO(1)],'r-','LineWidth',3);
    pltObs2(3) = plot([curpos0(2) VD(2)],[curpos0(1) VD(1)],'g-','LineWidth',3);
    if (nObspos > 0)
        pltObs2(4) = plot(obspos(1:nObspos,2),obspos(1:nObspos,1),'r*');
        for i = 1:nObspos
            pltObs2(4+i) = plot([curpos0(2) obspos(i,2)],[curpos0(1) obspos(i,1)],'k:');
        end
    end

    % Termination ---------------------------------------------------------
    drawnow;
    gErr2 = sqrt(((curpos0(1)-goal(1))*(curpos0(1)-goal(1))) + ((curpos0(2)-goal(2))*(curpos0(2)-goal(2))));
    if (gErr2 <= goalRds) 
        try if (~isempty(pltObs2)) set(pltObs2,'xData',NaN,'yData',NaN); end 
        catch if (~isempty(pltObs2)) delete(pltObs2); end
        end
        try if (~isempty(pltPath)) set(pltPath,'xData',NaN,'yData',NaN); end 
        catch if (~isempty(pltPath)) delete(pltPath); end
        end
        pltObs2 = plot([curpos0(2) VD(2)],[curpos0(1) VD(1)],'k-','LineWidth',3);
        break;
    end
    curpos0 = curpos;
end
