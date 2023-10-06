# TrussDeformation

Please see example image in this repository for an example of how to break down a truss into the 4 input vectors required by this program.

forces = 1 x n vector of each force acting on each node, input 0 if no force is acting on the node. Each force is given in pairs, forces = [2 1] indicates 2 units of force in the x direction and 1 unit of force in the y direction at node 1
DOF = 1 x n vector of the degrees of freedom for each node, 1 indicates no restriction and 0 indicates restriction. Each node is given in pairs, DOF = [1 0] indicates the node is free to move in the x direction but restricted in the y
nodes = 2 x n vector of the x and y coordinates of each node of the form nodes = [x1 y1; ,,, ; xn yn]
elements = 4 x n vector of the elements connecting each node of the form [node1 node2 ModofElasticity1 CrossSectionArea1; ... ; noden noden ModofElasticityn CrossSectionArean]
