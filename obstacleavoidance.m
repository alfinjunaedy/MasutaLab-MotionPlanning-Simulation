function [outDir,outWeight,outObspos,outNobspos] = obstacleavoidance(curpos,goal,wallnodes,obsnodes,safeDist)
    % MEX eval: obstacleavoidance([0 0],[0 0.5],{[1 1; 2 2; 3 3],[4 4; 5 5; 6 6]},{[1 1; 2 2; 3 3],[4 4; 5 5; 6 6]},0.1);
    outDir = atan2d(goal(2)-curpos(2),goal(1)-curpos(1));
    outNobspos = 0;
    outWeight = 0;
    dLo = 0.001;
    wOAss = 0.1;    
    outObspos = zeros(2000,2); 
    obsDir = zeros(2000,1);
    obsDist = zeros(2000,1);
    for i = 1:length(wallnodes)
        for j = 1:length(wallnodes{i})
            r = sqrt(((curpos(1)-wallnodes{i}(j,1))*(curpos(1)-wallnodes{i}(j,1))) + ((curpos(2)-wallnodes{i}(j,2))*(curpos(2)-wallnodes{i}(j,2))));
            if (r <= safeDist)
                outNobspos = outNobspos + 1;
                outObspos(outNobspos,:) = wallnodes{i}(j,:);
                obsDir(outNobspos) = atan2d(wallnodes{i}(j,2)-curpos(2),wallnodes{i}(j,1)-curpos(1));
                obsDist(outNobspos) = r;
            end
        end
    end
    for i = 1:length(obsnodes)
        for j = 1:length(obsnodes{i})
            r = sqrt(((curpos(1)-obsnodes{i}(j,1))*(curpos(1)-obsnodes{i}(j,1))) + ((curpos(2)-obsnodes{i}(j,2))*(curpos(2)-obsnodes{i}(j,2))));
            if (r <= safeDist)
                outNobspos = outNobspos + 1;
                outObspos(outNobspos,:) = obsnodes{i}(j,:);
                obsDir(outNobspos) = atan2d(obsnodes{i}(j,2)-curpos(2),obsnodes{i}(j,1)-curpos(1));
                obsDist(outNobspos) = r;
            end
        end
    end
    if (outNobspos > 0)
        curdir = atan2d(goal(2)-curpos(2),goal(1)-curpos(1));
        obsDir0 = obsDir(1:outNobspos) - curdir;
        obsDir0(obsDir0 > 180) = obsDir0(obsDir0 > 180) - 360; 
        obsDir0(obsDir0 < -180) = obsDir0(obsDir0 < -180) + 360;
        % Directional partitioning to F-B-R-L and Fr-Fl
        obsFBRL = obsDir0;
        obsFBRL(abs(obsDir0) <= 45) = 0;                % Front
        obsFBRL(abs(obsDir0) >= 135) = 1;               % Back
        obsFBRL(obsDir0 > 45 & obsDir0 < 135) = 2;      % Right
        obsFBRL(obsDir0 < -45 & obsDir0 > -135) = 3;    % Left
        obsFrFl = zeros(outNobspos,1); obsFrFl(:) = -1;
        obsFrFl(obsDir0 >= 0 & obsDir0 <= 90) = 0;      % Front-right
        obsFrFl(obsDir0 < 0 & obsDir0 >= -90) = 1;      % Front-left
        obsDist(outNobspos+1:2000) = Inf;
        [dmin,id] = min(obsDist);
        % Rules
        if ((any(obsFBRL==2) && any(obsFBRL==3)) || (any(obsFrFl==0) && any(obsFrFl==1)))
            a = circularAverage(90,-90,curdir);
            obsDistR = obsDist; obsDistR(obsFBRL ~= 2) = [];
            obsDistL = obsDist; obsDistL(obsFBRL ~= 3) = [];
            [dR,~] = min(obsDistR);
            [dL,~] = min(obsDistL);            
            flg = 1;
            if (dR-dL < 0) flg = -1; end
            outDir = a - (wOAss * (a - flg * 90));
            outWeight = 1.0;
        elseif ((any(obsFBRL==0) && any(obsFBRL==1) && ~any(obsFBRL==2) && ~any(obsFBRL==3)))
            a = circularAverage(0,180,curdir);
            obsDistF = obsDist; obsDistF(obsFBRL ~= 0) = [];
            obsDistB = obsDist; obsDistB(obsFBRL ~= 1) = [];
            [dF,~] = min(obsDistF);
            [dB,~] = min(obsDistB);            
            flg = 1;
            if (dF-dB < 0) flg = -1; end
            outDir = a - (wOAss * (a - flg * 180));
            outWeight = 1.0;
        else
            outWeight = 1.0 - ((dmin-dLo)/(safeDist-dLo));
            if (obsFBRL(id) == 2 || obsFrFl(id) == 0) outDir = -90 + curdir;
            elseif (obsFBRL(id) == 3 || obsFrFl(id) == 1) outDir = 90 + curdir;
            else outDir = 0 + curdir;
            end
        end
        if (outDir > 180) outDir = outDir - 360;
        elseif (outDir < -180) outDir = outDir + 360; 
        end
    end
end

function [out] = circularAverage(ang1,ang2,ang3)
    ang1 = ang1 + 1e-9; % Avoid 0 average
    x1 = cosd(ang1); y1 = sind(ang1);
    x2 = cosd(ang2); y2 = sind(ang2);
    x3 = cosd(ang3); y3 = sind(ang3);
    aveX = (x1 + x2 + x3) / 3.0;
    aveY = (y1 + y2 + y3) / 3.0;
    aveA = atan2d(aveY, aveX);
	out = aveA;
end
