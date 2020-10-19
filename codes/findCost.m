function cost = findCost(path, edges)
    % this function calculates the path cost.
    if length(path) == 2
        [r,~] = find(edges(:,1) == path(end));
        cost = edges(r,3);
    else        
        [r,~] = find(edges(:,1) == path(end));
        cost = edges(r,3) + findCost(path(1:end-1), edges);
    end
end        