% Write Description

function new_q = new_conf(rand_node,nearest_node,max_dist,k)
    d = sqrt((rand_node(1,2)-nearest_node(1,2))^2 + (rand_node(1,3)-nearest_node(1,3))^2);
    m = (nearest_node(1,3)-rand_node(1,3))/(nearest_node(1,2)-rand_node(1,2));
    if d <= max_dist
        new_q = [rand_node];
    else         
       if rand_node(1,2) > nearest_node(1,2)
           max_distx = max_dist;
       else 
           max_distx = -max_dist;
       end

       if rand_node(1,3) > nearest_node(1,3)
           max_disty = max_dist;
       else 
           max_disty = -max_dist;
       end 
                   
        if m == 0
           new_qx = nearest_node(1,2) + max_distx;
           new_qy = nearest_node(1,3);
           new_q = [k,new_qx,new_qy];
                      
        elseif abs(m) == inf 
           new_qx = nearest_node(1,2);
           new_qy = nearest_node(1,3) + max_disty;
           new_q = [k,new_qx,new_qy];
           
        else
               if rand_node(1,2) > nearest_node(1,2)
                   max_distx = max_dist;
               else 
                   max_distx = -max_dist;
               end
               
               if rand_node(1,3) > nearest_node(1,3)
                   max_disty = max_dist;
               else 
                   max_disty = -max_dist;
               end
        new_qx = nearest_node(1,2)+ max_distx*sqrt(1/(1+m^2));
        new_qy = nearest_node(1,3)+ abs(m)*max_disty*sqrt(1/(1+m^2));     
        new_q = [k,new_qx,new_qy];        
        end
    end