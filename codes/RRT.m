clc; clear;  % Clear the command window and the workspace first.\

% Import data and Initialization
% Most of the Initialization are optimized for the fastest performance. The
% default settings are suggested.
obstacles = readmatrix('D:\Matlab 2020\Peer Assignments\Mine\Course4_Week2\RRT-Path-Finder\result\obstacles.csv');
start = [-0.5, -0.5];
goal =[0.5, 0.5]; % The start and goal configuration can be changed according
                                     % to user preference.
isFound = 0; % the boolean to state if the path is found before maximum iterations.
goal_area = 0.2; % The goal area determines the goal region for fast and better performance.
goal_area(goal_area > 0.2) = 0.2;
itr = 300; % The itr is the maximum iterration for finding the path.
max_dist = 0.09; % Max_dist is the maximum distance from the nearest node to the new configuration node.
max_dist(max_dist > 0.09) = 0.09; % Limit the max_dist to 0.1.
edges = zeros(itr, 3);
rand_node =[];
nodes = [1, start(1), start(2), euclidean_dist(start, goal)];
figure;
axis equal;

% RRT Alogorithm

for current = 2 : itr + 1
    rand_node = [];
    while isempty(rand_node)
        rand_node = rand_conf(obstacles, current);     % find the obstacle free rand_node.
        nearest_node = local_planner(nodes, rand_node);  % find the nearest node in the 'nodes' matrix.
        new_node = new_conf(rand_node, nearest_node, max_dist, current); %place a new node at the max_dist in the rand_node direction
        
        if checkcollision_point(obstacles, new_node) % check the collision for new_node
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

%Post algorithm (for data cleaning and visual experiece)

nodes(end+1, :)= [nodes(end, 1) + 1, goal, euclidean_dist(goal, goal)];
nearest_node = local_planner(nodes(2:end, :), nodes(1, :));
edges(1, :) = [nodes(1, 1), nearest_node(1), euclidean_dist(nearest_node, nodes(1,:))];

nearest_node = local_planner(nodes(1 : end-1, :), nodes(end, :));
edges(current + 1, :) = [nodes(end, 1), nearest_node(1), euclidean_dist(nearest_node, nodes(end, :))];
edges = edges(any(edges, 2), :);

if checkcollision_edge(nodes(end, :), nearest_node, obstacles)
    % checks the collision between the nearest_node possible and the goal:
    % if iscollide then path == fail else path = construct_path.
    disp("fatal: The path is not found, suggest you to add more iterations");
end
path = construct_path(edges);
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