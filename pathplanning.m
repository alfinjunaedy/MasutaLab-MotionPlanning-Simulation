function [outPath,outNpath] = pathplanning(start,goal,wallnodes,obsnodes,safeDist,pathStep,timeout)
    % MEX eval: pathplanning([0 0],[0 0.5],{[1 1; 2 2; 3 3],[4 4; 5 5; 6 6]},{[1 1; 2 2; 3 3],[4 4; 5 5; 6 6]},0.1,0.1,3);
    tic;    
    outPath = zeros(2000,2); 
    outNpath = 0;
    waitList = zeros(2000,2); 
    nList = 0;
    obsDetected = false;
    dGoal = sqrt(((start(1)-goal(1))*(start(1)-goal(1)))+((start(2)-goal(2))*(start(2)-goal(2))));
    steps = pathStep/dGoal:pathStep/dGoal:1;
    for i = 1:length(steps)
        ii = steps(i);
        o = start + ii * (goal-start);
        % Check for obstacles
        obsDetected = ~obstaclefree(o,wallnodes,obsnodes,safeDist);
        if (obsDetected == true) break; end
        nList = nList + 1;
        waitList(nList,:) = o;
    end
    if (obsDetected == false)
        outNpath = nList;
        outPath(1:outNpath,:) = waitList(1:nList,:);  
    else
        iList = 0; goalDetected = 0;
        if (nList == 0)
            nList = nList + 1;
            waitList(nList,:) = start;
        end  
        while (goalDetected == 0)
            iList = iList + 1;
            if (iList > nList) break; end
            bestD = [9999 9999 9999 9999]; bestXY = zeros(length(bestD),2);
            for i = 1:8
                if (i == 1) xy = [pathStep -pathStep];
                elseif (i == 2) xy = [pathStep 0];
                elseif (i == 3) xy = [pathStep pathStep];
                elseif (i == 4) xy = [0 -pathStep];
                elseif (i == 5) xy = [0 pathStep];
                elseif (i == 6) xy = [-pathStep -pathStep];
                elseif (i == 7) xy = [-pathStep 0];
                else xy = [-pathStep pathStep];
                end
                pos = [waitList(iList,1)+xy(1) waitList(iList,2)+xy(2)];
                skip = 0;
                if (start(1) == pos(1) && start(2) == pos(2)) continue; end
                if (skip == 1) continue; end
                for j = 1:nList
                    if (abs(waitList(j,1) - pos(1)) < pathStep*0.5)
                        if (abs(waitList(j,2) - pos(2)) < pathStep*0.5)
                            skip = 1;
                            break;
                        end
                    end
                end
                if (skip == 1) continue; end
                free = obstaclefree(pos,wallnodes,obsnodes,safeDist);
                if (free == 0) continue; end
                d = sqrt(((pos(1)-goal(1))*(pos(1)-goal(1))) + ((pos(2)-goal(2))*(pos(2)-goal(2))));
                for j = 1:length(bestD)
                    if d < bestD(j)
                        for k = length(bestD):-1:(j+1)
                            bestD(k) = bestD(k-1);
                            bestXY(k,:) = bestXY(k-1,:);
                        end
                        bestD(j) = d;
                        bestXY(j,:) = pos;
                        break;
                    end
                end
            end
            for i = 1:length(bestD)
                if (bestD(i)>=9999) break; end
                % Check if goal can be seen
                dGoal = sqrt(((bestXY(i,1)-goal(1))*(bestXY(i,1)-goal(1)))+((bestXY(i,2)-goal(2))*(bestXY(i,2)-goal(2))));
                steps = pathStep/dGoal:pathStep/dGoal:1;
                for j = 1:length(steps)
                    jj = steps(j);
                    o = bestXY(i,:) + jj * (goal-bestXY(i,:));
                    goalDetected = obstaclefree(o,wallnodes,obsnodes,safeDist);
                    if (goalDetected == 0) break; end
                end
                nList = nList + 1;
                waitList(nList,:) = bestXY(i,:);
                if (goalDetected == 1) break; end
            end
            if (toc >= timeout) break; end
        end
        if (goalDetected == 1)
            outNpath = 1;
            outPath(outNpath,:) = goal;
            outNpath = outNpath + 1;
            outPath(outNpath,:) = waitList(nList,:);
            oldPathN = 1;
            oldPathIdx = zeros(2000,1); 
            oldPathIdx(1) = 1;
            startDetected = 0;
            while 1
                if (toc >= timeout) break; end
                dBest = -1; xyBest = zeros(1,2); iBest = 0;
                for i = 1:nList
                    if (waitList(i,1) == outPath(outNpath,1) && waitList(i,2) == outPath(outNpath,2)) continue; end
                    d = sqrt(((waitList(i,1)-outPath(outNpath,1))*(waitList(i,1)-outPath(outNpath,1))) + ((waitList(i,2)-outPath(outNpath,2))*(waitList(i,2)-outPath(outNpath,2))));
                    % 1) Search within radius
                    if (d <= pathStep*5)
                        free = 1;
                        steps = pathStep/d:pathStep/d:1;
                        for j = 1:length(steps)
                            jj = steps(j);
                            o = waitList(i,:) + jj * (outPath(outNpath,:)-waitList(i,:));
                            free = obstaclefree(o,wallnodes,obsnodes,safeDist);
                            if (free == 0) break; end
                        end
                        if (free == 1)
                            % 2) Free cur-candidate
                            dStart = sqrt(((waitList(i,1)-start(1))*(waitList(i,1)-start(1))) + ((waitList(i,2)-start(2))*(waitList(i,2)-start(2))));
                            steps = pathStep/dStart:pathStep/dStart:1;
                            for j = 1:length(steps)
                                jj = steps(j);
                                o = waitList(i,:) + jj * (start-waitList(i,:));
                                startDetected = obstaclefree(o,wallnodes,obsnodes,safeDist);
                                if (startDetected == 0) break; end
                            end
                            % 3) Best candidate
                            if (d > dBest || startDetected == 1)
                                skip = 0;
                                for j = 1:oldPathN
                                    if (i == oldPathIdx(j))
                                        skip = 1;
                                        break;
                                    end
                                end
                                if (skip == 0 || startDetected == 1)
                                    dBest = d;
                                    xyBest = waitList(i,:);
                                    iBest = i;
                                end
                            end
                        end
                    end
                    if (startDetected == 1) break; end
                end
                if (startDetected == 1) 
                    outNpath = outNpath + 1;
                    outPath(outNpath,:) = xyBest;
                    oldPathN = oldPathN + 1;
                    oldPathIdx(oldPathN) = iBest;
                    break; 
                end
                if (iBest == 0) continue; end
                % 4) Backward search for the shortest distance
                free = 0;
                for i = 1:outNpath-1
                    d = sqrt(((xyBest(1)-outPath(i,1))*(xyBest(1)-outPath(i,1))) + ((xyBest(2)-outPath(i,2))*(xyBest(2)-outPath(i,2))));
                    steps = pathStep/d:pathStep/d:1;
                    for j = 1:length(steps)
                        jj = steps(j);
                        o = xyBest + jj * (outPath(i,:)-xyBest);
                        free = obstaclefree(o,wallnodes,obsnodes,safeDist);
                        if (free == 0) break; end
                    end
                    if (free == 1)
                        ii = outNpath;
                        for j = ii:-1:i+1
                            if (outNpath <= 1) break; end
                            outPath(j,:) = zeros(1,2);
                            outNpath = outNpath - 1;
                        end
                        break; 
                    end
                end
                % 5) Update array
                outNpath = outNpath + 1;
                outPath(outNpath,:) = xyBest;
                oldPathN = oldPathN + 1;
                oldPathIdx(oldPathN) = iBest;
            end
            if (startDetected == 1)
                stepB = 0;
                while 1
                    stepA = 0; iMax = 1;
                    for i = 1:outNpath-2
                        iMax = i;
                        stepA = stepA + 1;
                        if (stepA <= stepB) continue; end
                        free = 0;
                        for j = outNpath:-1:outNpath+2
                            d = sqrt(((outPath(i,1)-outPath(j,1))*(outPath(i,1)-outPath(j,1))) + ((outPath(i,2)-outPath(j,2))*(outPath(i,2)-outPath(j,2))));
                            steps = pathStep/d:pathStep/d:1;
                            for k = 1:length(steps)
                                kk = steps(k);
                                o = outPath(i,:) + kk * (outPath(j,:)-outPath(i,:));
                                free = obstaclefree(o,wallnodes,obsnodes,safeDist*1.5);
                                if (free == 0) break; end
                            end
                            if (free == 1) 
                                stepB = stepA;
                                for k = j-1:-1:i+1
                                    outPath(1+stepA,:) = zeros(1,2);
                                    for l = 1+stepA:outNpath-1
                                        outPath(l,:) = outPath(l+1,:);
                                    end
                                    outNpath = outNpath - 1;
                                end
                                break; 
                            end
                        end
                        if (free) break; end
                    end
                    if (iMax >= outNpath-2) break; end
                    if (toc >= timeout) break; end
                end
                outPath(1:outNpath,:) = outPath(outNpath:-1:1,:);
            else
                outNpath = 0;
            end
        end
    end
end

function [out] = obstaclefree(pos,wallnodes,obsnodes,safeDist)
    free = 1;
    for i = 1:length(wallnodes)
        for j = 1:length(wallnodes{i})
            r = sqrt(((pos(1)-wallnodes{i}(j,1))*(pos(1)-wallnodes{i}(j,1))) + ((pos(2)-wallnodes{i}(j,2))*(pos(2)-wallnodes{i}(j,2))));
            if (r <= safeDist)
                free = 0;
                break;
            end
        end
        if (free == 0) break; end
    end
    if (free == 1)
        for i = 1:length(obsnodes)
            for j = 1:length(obsnodes{i})
                r = sqrt(((pos(1)-obsnodes{i}(j,1))*(pos(1)-obsnodes{i}(j,1))) + ((pos(2)-obsnodes{i}(j,2))*(pos(2)-obsnodes{i}(j,2))));
                if (r <= safeDist)
                    free = 0;
                    break;
                end
            end
            if (free == 0) break; end
        end
    end
    out = free;
end
