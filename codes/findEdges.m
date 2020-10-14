% Write description

function edges = findEdges(current,nodes,edges)
distance = zeros(1,size(nodes,1));
for i =1:size(nodes,1)
    Euclidean_dist = sqrt((current(1,2)-nodes(i,2))^2 +(current(1,3)-nodes(i,3))^2);
    distance (1,i) = Euclidean_dist;
end
data = distance;
distance = sort(distance,2);
distance = distance(distance<0.1);

if length(distance)<=3
   for k =1:length(distance)
       [~,c] = find(data == distance(k));
       temp = [nodes(c,1),current(1),distance(k)];
       edges = vertcat(edges,temp);      
   end
else 
    for k =1:3
        [~,c] = find(data == distance(k));
        temp = [nodes(c,1),current(1),distance(k)];
        edges = vertcat(edges,temp);
    end
end
    
