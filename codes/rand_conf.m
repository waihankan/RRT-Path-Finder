% In RRT, "RAND_CONF" grabs a random configuration qrand in C. This may be replaced with a function 
%"RAND_FREE_CONF" that uses samples in Cfree, while rejecting those in Cobs
% using some collision detection algorithm.

% Write Description!
function rand_node = rand_conf(obstacles,k)
rand_node = [];
while isempty(rand_node)
    % limits
    min_boundary = -0.5; max_boundary = 0.5;              
    r_int = (max_boundary-min_boundary).*rand(1,2) + min_boundary;
    for i = 1:size(obstacles,1)
        d = sqrt((r_int(1,1)-obstacles(i,1))^2 + (r_int(1,2)-obstacles(i,2))^2);
        if d <=obstacles(i,3)
            rand_node =[];
            break;
        end
                        
    rand_node= [k,r_int(1,1),r_int(1,2)];    
    end
end
    
