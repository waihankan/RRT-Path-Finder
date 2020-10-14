% this file is to build a module that check collisions with all the
% obstacles one by one.

function collide = edge_collision(new_node,nearest_node,obstacles)

for i = 1:size(obstacles,1)
    flag_intersect = collision(new_node,nearest_node,obstacles(i,:));
    if flag_intersect          
        collide = true; % return that the path is in collision
        break; 
    end                        
    collide = false;
end
end
