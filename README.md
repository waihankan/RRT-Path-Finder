# RRT-Path-Finder
=================
Modules loaded for Rapidly Exploring Random Tress Algorithm

An RRT grows a tree rooted at the starting configuration by using random samples from the search space. As each sample is drawn, a connection is attempted between it and the nearest state in the tree. If the connection is feasible (passes entirely through free space and obeys any constraints), this results in the addition of the new state to the tree.

In my code, I make a goal area for the Algorithm much faster and more efficient. This makes you to get to the goal in most (approximately 80%)of the time.

For visualization, I plot in the MATLAB. If you are not a MATLAB  user, you may need to implement yourself to figure out what's happening.

Functions I used are included in the codes folder.

The result folder is an example of RRT path finding Algorithm.

Note : You may want to change the some of the code for your use. By default, I choose most of the values to optimal values so that the code run fast and efficient.

For simulation, if you use Coppelia_Sim software, you just browse the result folder and run it.
