% Find the nearest node of the current node.

function nearest_node = local_planner(nodes,rand_node)

distance = zeros(1,size(nodes,1));
for i = 1:size(nodes,1)
    Euclidean_dist = sqrt((rand_node(1,2)-nodes(i,2))^2 +(rand_node(1,3)-nodes(i,3))^2);
    distance(1,i) = Euclidean_dist;
end
% distance = sort(distance,2);
% distance = distance(:,1:3)
[~,c] = find(distance == min(distance));
nearest_node = [nodes(c,1),nodes(c,2),nodes(c,3)];

end



    
    