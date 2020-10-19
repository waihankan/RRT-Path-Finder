clc; clear;

% Import data and Initialization

obstacles = readmatrix('obstacles.csv');
obstacles(:, 3) = obstacles(:, 3)/2; % diameter to radius.
start = [-0.5, -0.5];
goal =[0.5, 0.5];
isFound = 0; 
goal_area = 0.25; % bias the goal.

itr = 100;
max_dist = 0.2;
edges = zeros(itr, 3);
rand_node =[];
nodes = [1, start(1), start(2), euclidean_dist(start, goal)];

% For figure

f1 = figure;
axis equal;
set(gca,'XTick', -0.5 : 0.1 : 0.5);
set(gca,'YTick', -0.5 : 0.1 : 0.5);
xlim([-0.5 0.5]);
ylim([-0.5 0.5]);

% RRT Alogorithm

for current = 2 : itr + 1
    rand_node = [];
    
    while isempty(rand_node)
        rand_node = rand_conf(obstacles, current);     
        nearest_node = local_planner(nodes, rand_node); 
        new_node = new_conf(rand_node, nearest_node, max_dist, current);
        
        %check the new node for collision.
        if checkcollision_point(obstacles, new_node) || ...
           isCollision(new_node, nearest_node, obstacles)
            rand_node =[];  
            
        else                          
            nodes(current,:) = [new_node, euclidean_dist(new_node, goal)];
            edges(current,:) = [new_node(1), nearest_node(1), euclidean_dist(nearest_node, new_node)];
            % plotting
            hold on;
            plot(nearest_node(2), nearest_node(3), 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
            plot(new_node(2), new_node(3), 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r'); 
            plot([nearest_node(2); new_node(2)], [nearest_node(3); new_node(3)], '-k');
            
            if euclidean_dist(new_node, goal) < goal_area
                isFound = 1;
                fprintf("The path is found after %d iterations.\n" , current);
                break;
            end
        end
    end
    
    if isFound
        break;
    end
end

%Post algorithm (reconnect and visual experiece)

nodes(end+1, :)= [nodes(end, 1) + 1, goal, euclidean_dist(goal, goal)];
nearest_node = local_planner(nodes(2:end, :), nodes(1, :));
edges(1, :) = [nodes(1, 1), nearest_node(1), euclidean_dist(nearest_node, nodes(1,:))];

nearest_node = local_planner(nodes(1 : end-1, :), nodes(end, :));
edges(current + 1, :) = [nodes(end, 1), nearest_node(1), euclidean_dist(nearest_node, nodes(end, :))];
edges = edges(any(edges, 2), :);

if isCollision(nodes(end, :), nearest_node, obstacles)
    % check collision from goal to nearest node.
    close(f1);
    error("fatal: The path is not found, add more iterations or increase the distance from node to node.");
end

% path reconstruction and calculate cost.
path = construct_path(edges);
path_cost = findCost(path, edges);
fprintf("Success. \n");
fprintf("Total cost to reach goal : %.4f \n", path_cost);

hold on;
plot(nearest_node(2), nearest_node(3), 'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
plot(nodes(end, 2), nodes(end, 3), 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r'); 
plot([nearest_node(2); nodes(end, 2)], [nearest_node(3); nodes(end, 3)], '-k');

for i = 1 : size(obstacles, 1)
    circle(obstacles(i, 1), obstacles(i, 2), obstacles(i, 3));
end

% Output files as csv files.
writematrix(edges, 'edges.csv');
writematrix(nodes, 'nodes.csv');
writematrix(path, 'path.csv');