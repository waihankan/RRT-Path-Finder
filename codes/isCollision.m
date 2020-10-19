function collision = isCollision(point1,point2, obstacles)
    point1 = point1(2:3);
    point2 = point2(2:3);
    for i = 1 : size(obstacles, 1)        
        if Col(point1, point2, obstacles(i, :))
            collision = true;
            break;
        end
        
        collision = false; 
    end
end

function collision = Col(point1, point2, obstacle)
    % parmetric equation of circle
    % x = h + r*cos(t)
    % y = k + r*sin(t)
    % theta = 0 to 2pi
    
    theta = 0 : 0.01 : 2*pi;
    radius = obstacle(3) + 0.001;
    x = obstacle(1) + radius * cos(theta);
    y = obstacle(2) + radius * sin(theta);
    X1 = [point1(1); point2(1)];
    Y1 = [point1(2); point2(2)];    
    [x0, y0] = intersections (X1, Y1, x, y, true);
    
    if isempty([x0, y0])
        collision = false;
    else
        collision = true;
    end
end
