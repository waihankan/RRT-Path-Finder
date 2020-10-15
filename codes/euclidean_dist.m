function distance = euclidean_dist(start,goal)
    % calculates the distance between the two nodes.
    if size(start,2) > 2
        start = start(1,2:end);
    end
    if size(goal, 2) > 2
        goal = goal(1,2:end);
    end
    distance = sqrt((goal(1) - start(1))^2 + (goal(2) - start(2))^2);
end

        
        
    
    