function path = construct_path(edges)
    % this function retraces the path.
    current = edges(end,1);
    start = edges(1,1);
    parent = zeros(1,size(edges,1));
    i =2;
    
    while current ~= start
        
        [r,c] = find(edges(:,1) == current);
        parent(1,i) = edges(r,c+1);
        current = parent(1,i);
        i = i +1;
        
    end
    
    parent(1,1) = edges(end,1);
    parent( :, ~any(parent,1) ) = [];    
    path = flip(parent);
    
end
        