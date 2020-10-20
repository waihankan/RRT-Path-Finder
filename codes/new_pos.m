function new_q = new_pos(rand_node,nearest_node,max_dist,k)
    P1 = nearest_node(2:3);
    P2 = rand_node(2:3);

    %Normalize the vector
    v = P2 - P1;
    unitv =  v/norm(v);
    
    %Get a point from p1 in unitv direction at a distance "max_dist"
    P0 = P1 + (max_dist * unitv);
    new_q = horzcat(k, P0);
end