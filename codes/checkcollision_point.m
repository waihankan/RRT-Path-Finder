%Write Description

function collision = checkcollision_point(obstacles,node)
    for i = 1:size(obstacles,1)
        d = sqrt((node(1,2)-obstacles(i,1))^2 + (node(1,3)-obstacles(i,2))^2);
        if d < obstacles(i,3)
           collision = true;
           break;
        end
        collision = false;
    end
end