clc;clear;  % Clear the command window and the workspace first.\

% Import data and Initialization
% Most of the Initialization are optimized for the fastest performance. The
% default settings are suggested.
obstacles = readmatrix('obstacles_filename');
start = [-0.5,-0.5]; goal =[0.5,0.5]; % The start and goal configuration can be changed according
                                     % to user preference.
solution =0;
goal_area = 0.2; % The goal area determines the goal region for fast and better performance.
itr = 300; % The itr is the maximum iterration for finding the path.
max_dist = 0.08; % Max_dist is the maximum distance from the nearest node to the new configuration node.
edges = zeros(itr,3);
rand_node =[];
nodes = [1, start(1), start(2), Euclidean(start,goal)];
figure;
axis equal;

% RRT Alogorithm

for k = 2:itr+1
    rand_node = [];
    while isempty(rand_node)
        rand_node = rand_conf(obstacles, k);     %find the obstacle free rand_node.
        nearest_node = local_planner(nodes, rand_node);  %find the nearest node in the 'nodes' matrix.
        new_node = new_conf(rand_node, nearest_node, max_dist, k); %place a new node at the max_dist in the rand_node direction
        collision_pt = collisionPoint(obstacles,new_node); %check the collision for new_node
        if collision_pt
            rand_node =[];
        else                          
            nodes(k,:) = [new_node, Euclidean([new_node(2), new_node(3)], goal)];
            edges(k,:) = [new_node(1), nearest_node(1), sqrt((nearest_node(2)-new_node(2))^2+(nearest_node(3)-new_node(3))^2)];
            % plotting
            hold on;
            plot(nearest_node(2), nearest_node(3),'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
            plot(new_node(2),new_node(3), 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r'); 
            plot([nearest_node(2);new_node(2)], [nearest_node(3); new_node(3)], '-k');
            if sqrt((new_node(2) - goal(1))^2 + (new_node(3) - goal(2))^2) < goal_area
                solution = 1;
                fprintf("The path is found after %d iterations.\n" , k);
                break;
            end
        end
    end
    if solution
        break;
    end
end

%Post algorithm (for data cleaning and visual experiece)

nodes(end+1,:)= [nodes(end,1)+1,goal,Euclidean(goal,goal)];

nearest_node = local_planner(nodes(2:end,:),nodes(1,:));
edges(1,:) = [nodes(1,1),nearest_node(1),sqrt((nearest_node(2)-nodes(1,2))^2+(nearest_node(3)-nodes(1,3))^2)];

nearest_node = local_planner(nodes(1:end-1,:),nodes(end,:));

edges(k+1,:) = [nodes(end,1),nearest_node(1),sqrt((nearest_node(2)-nodes(end,2))^2+(nearest_node(3)-nodes(end,3))^2)];
edges = edges(any(edges,2),:);

collide = edge_collision(nodes(end,:),nearest_node,obstacles);
if collide
    disp("The path is not found, suggest you to add more iterations");
end
path = construct_path(edges);
hold on;
plot(nearest_node(2), nearest_node(3),'o', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
plot(nodes(end,2),nodes(end,3), 'o', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r'); 
plot([nearest_node(2);nodes(end,2)], [nearest_node(3); nodes(end,3)], '-k');

for i = 1:size(obstacles,1)
    circle(obstacles(i,1),obstacles(i,2),obstacles(i,3));
end

% Output files as csv files.
writematrix(edges,'edges.csv')
writematrix(nodes,'nodes.csv')
writematrix(path,'path.csv')