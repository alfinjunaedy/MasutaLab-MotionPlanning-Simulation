function [outObsnodes] = updateobstacle(obsStep,obsDist,wallnodes,obsnodes)
    % MEX eval: updateobstacle(0.01,0.1,{[1 1; 2 2; 3 3],[4 4; 5 5; 6 6]},{[1 1; 2 2; 3 3],[4 4; 5 5; 6 6]})
    obscntr = zeros(length(obsnodes),2);
    for i = 1:length(obsnodes)
        obscntr(i,:) = mean(obsnodes{i},1);
    end
    for i = 1:length(obsnodes)
        dir = -180 + 360 * rand();
        pos = [obsStep*cosd(dir) obsStep*sind(dir)];
        skip = 0;
        for j = 1:length(wallnodes)
            r = sqrt(((obscntr(i,1)+pos(1)-wallnodes{j}(:,1)).*(obscntr(i,1)+pos(1)-wallnodes{j}(:,1))) + ((obscntr(i,2)+pos(2)-wallnodes{j}(:,2)).*(obscntr(i,2)+pos(2)-wallnodes{j}(:,2))));
            if (any(r <= obsDist))
                skip = 1;
                break;
            end
        end
        if (skip == 1) continue; end
        for j = 1:length(obsnodes)
            if (j == i) continue; end
            r = sqrt(((obscntr(i,1)+pos(1)-obscntr(j,1))*(obscntr(i,1)+pos(1)-obscntr(j,1))) + ((obscntr(i,2)+pos(2)-obscntr(j,2))*(obscntr(i,2)+pos(2)-obscntr(j,2))));
            if (r <= obsDist) 
                skip = 1;
                break;
            end
        end
        if (skip == 1) continue; end
        obscntr(i,:) = obscntr(i,:) + pos;
        obsnodes{i} = obsnodes{i} + pos;
    end
    outObsnodes = obsnodes;
end