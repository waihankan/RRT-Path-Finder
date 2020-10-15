function iscollide = checkcollision_edge(new_node,nearest_node,obstacles)
    % The function checks whether the path between the new_node and the
    % nearest node is in collision or not.
    for i = 1:size(obstacles,1)
        flag_intersect = collision(new_node,nearest_node,obstacles(i,:));
        if flag_intersect          
            iscollide = true; % return that the path is in collision.
            break;
        end                        
        iscollide = false;
    end
end

function flag_intersect = collision(new_node,nearest_node,obstacles)

    xc = obstacles(1);yc = obstacles(2);       % centre of the circle
    R = obstacles(3);                         % is the radius of the circle
    x1 = new_node(2); y1 = new_node(3);            % 1st point of the segment
    x2 = nearest_node(2); y2 = nearest_node(3);    % 2nd point of the segment

    d1c = norm( [(x1-xc); (y1-yc)] );           % distance of P1 from the centre
    d2c = norm( [(x2-xc); (y2-yc)] );           % distance of P2 from the centre
    if (d2c<R) && (d1c<R)   
        flag_intersect = false ;

    elseif (d1c<R) && (d2c>R)
         flag_intersect = true;

    elseif (d2c<R) && (d1c>R)
        flag_intersect = true;

    else
        P12 = [(x2-x1) ; (y2-y1)];
        uP12 = P12/norm(P12);
        P1C = [(xc - x1); (y2 - yc)];
        v = abs( uP12(1)*P1C(2) - uP12(2)*P1C(1) );
        flag_intersect = (v < R);

    end
end