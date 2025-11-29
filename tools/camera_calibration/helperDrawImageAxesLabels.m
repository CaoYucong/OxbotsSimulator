function [originLabel,xLabel,yLabel] = helperDrawImageAxesLabels(boardSize,imagePoints)

    numBoardRows = boardSize(1)-1;
    numBoardCols = boardSize(2)-1;
    
    % Reshape checkerboard corners to boardSize shaped array
    boardCoordsX = reshape(imagePoints(:,1), [numBoardRows,numBoardCols]);
    boardCoordsY = reshape(imagePoints(:,2), [numBoardRows,numBoardCols]);
    boardCoords = cat(3, boardCoordsX,boardCoordsY);
    
    % Origin label (check if origin location is inside the image)
    if ~isnan(boardCoordsX(1,1))
        p1 = boardCoords(1,1,:);
        
        refPointIdx = find(~isnan(boardCoordsX(:,1)),2);
        p2 = boardCoords(refPointIdx(2),1,:);
        
        refPointIdx = find(~isnan(boardCoordsX(1,:)),2);
        p3 = boardCoords(1,refPointIdx(2),:);
        
        [loc, theta] = getAxesLabelPosition(p1,p2,p3);
        
        originLabel.Location = loc;
        originLabel.Orientation = theta;
    else
        originLabel = struct;
    end
    
    % X-axis label
    firstRowIdx = numBoardCols:-1:1;
    refPointIdx13 = find(~isnan(boardCoordsX(1,firstRowIdx)),2);
    refPointIdx13 = firstRowIdx(refPointIdx13);
    
    p1 = boardCoords(1,refPointIdx13(1),:);
    p3 = boardCoords(1,refPointIdx13(2),:);
    
    refPointIdx2 = find(~isnan(boardCoordsX(:,refPointIdx13(1))),2);
    p2 = boardCoords(refPointIdx2(2),refPointIdx13(1),:);
    
    [loc, theta] = getAxesLabelPosition(p1,p2,p3);
    theta = 180 + theta;
    
    xLabel.Location    = loc;
    xLabel.Orientation = theta;
    
    % Y-axis label
    firstColIdx = numBoardRows:-1:1;
    refPointIdx12 = find(~isnan(boardCoordsX(firstColIdx,1)),2);
    refPointIdx12 = firstColIdx(refPointIdx12);
    
    p1 = boardCoords(refPointIdx12(1),1,:);
    p2 = boardCoords(refPointIdx12(2),1,:);
    
    refPointIdx3 = find(~isnan(boardCoordsX(refPointIdx12(1),:)), 2);
    p3 = boardCoords(refPointIdx12(1),refPointIdx3(2),:);
    
    [loc,theta] = getAxesLabelPosition(p1,p2,p3);
    
    yLabel.Location = loc;
    yLabel.Orientation = theta;
    
    %--------------------------------------------------------------
    % p1+v
    %  \
    %   \     v1
    %    p1 ------ p2
    %    |
    % v2 |
    %    |
    %    p3
    function [loc,theta] = getAxesLabelPosition(p1,p2,p3)
        v1 = p3 - p1;
        theta = -atan2d(v1(2),v1(1));
        
        v2 = p2 - p1;
        v = -v1 - v2;
        d = hypot(v(1),v(2));
        minDist = 40;
        if d < minDist
            v = (v / d) * minDist;
        end
        loc = p1 + v;
    end
    %--------------------------------------------------------------
    
end