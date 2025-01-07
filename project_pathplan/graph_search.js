/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function initSearchGraph() {
    // STENCIL: determine whether the graph node should be the start point for the search
    // create the search queue
    visit_queue = [];
    
    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false, // flag for whether the node has been queued for visiting
                costCalculated:false //check if a cost has been calculated for the cell
            };
            // STENCIL: determine whether the graph node should be the start or goal point for the search

            // bounds of grid array for grid square i,j
            let max_x = G[iind][jind].x + (eps/2);
            let min_x = G[iind][jind].x - (eps/2);
            let max_y = G[iind][jind].y + (eps/2);
            let min_y = G[iind][jind].y - (eps/2);

            //check if the start is within the grid bounds
            if(checkGridSquare(q_init, min_x, max_x, min_y, max_y)){
                visit_queue.push(G[iind][jind]);
                G[iind][jind].queued = true;
                G[iind][jind].distance = 0; 
            }

            //check if the goal is within the grid bounds
            else if(checkGridSquare(q_goal, min_x, max_x, min_y, max_y)){
                goal_index_x = iind;
                goal_index_y = jind;
                goal_node = G[goal_index_x][goal_index_y];
            }            
        }
    }


}

function iterateGraphSearch() {
    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   When search is complete ("failed" or "succeeded") set the global variable 
    //   search_iterate to false. 
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location

    if(search_alg == "A-star"){
        return A_star()
    }
    else if(search_alg == "depth-first"){ 
        return dfs()
    }
    else if(search_alg == "breadth-first"){ 
        return bfs();
    }
    else if(search_alg == "greedy-best-first"){ 
        return greedy_best_first_search();
    }
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.

function minheap_insert(new_element, heap){
    // find index of new element
    var elementIndex = heap.length;

    // find length of new elements parent
    var parentIndex = Math.floor((elementIndex-1)/2);
      
    // add new element to heap array
    heap.push(new_element);
       
    // initialize heap condition
    // heap condition is true if new element is added as root 
    // or if new element is less than or equal to its parent elements f_score
    var heaped = (elementIndex <= 0) || (heap[parentIndex].priority <= heap[elementIndex].priority);

    while(!heaped){

        //swap element and parent
        var tmp = heap[parentIndex];
        heap[parentIndex] = heap[elementIndex];
        heap[elementIndex] = tmp;

        // update element and parent index
        elementIndex = parentIndex;
        parentIndex = Math.floor((elementIndex-1)/2);

        // re-evaluate heap condition
        heaped = ((elementIndex <= 0) || (heap[parentIndex].priority <= heap[elementIndex].priority));
        
    }

}

function minheap_extract(heap){
    //if the last element is reached, return it
    if(heap.length == 1){
        extractedValue = heap.pop();
        return extractedValue;
    }

    // extract root element, then replace it with the last element
    extractedValue = heap[0];
    heap[0] = heap.pop();

    //swap with higher priority child (heapify)
    let index = 0; //index of new root element
    
    while(true){
               
        leftIndex = 2*index+1;
        rightIndex = 2*index+2;
        length = heap.length-1;
        childIndex = null; // smallest child index
        
        // determine priority child
        //if left child exists make it the smallest child index
        if(leftIndex <= length){//} && heap[leftIndex] < heap[index]){
            childIndex = leftIndex;
        }
         //if right child exists, compare it to the left index and update the child index
         if(rightIndex <= length && heap[rightIndex].priority < heap[leftIndex].priority){
            childIndex = rightIndex;
        }
        // if the inital root is smaller than both children then break, otherwise heapify repeatedly
        if (childIndex == null || heap[index].priority <= heap[childIndex].priority){
            break;
        }
        else {
            //swap index and child index
            var tmp = heap[index];
            heap[index] = heap[childIndex];
            heap[childIndex] = tmp;
            
            //[heap[index],heap[childIndex]] =[heap[childIndex], heap[index]];
            index = childIndex; //reassign the index of the element
        }
    }
    
    return extractedValue;
}

//////////////////////////////////////////////////
/////     STUDENT IMPLEMENTED FUNCTIONS
//////////////////////////////////////////////////



function findNeighbors(current_node, order = "ewsn"){
    /* 
        Given the current node, find all neighbors
        Default order of neighbors returned: East, West, South, North
        
        Input Parameters:
            current_node:
            order: (string) parameter to specify the order that the neighbors are discovered in
                Use letters n,e,s,w in a string to specify the order
    */

    // Empty vector
    let neighbors = [];

    // getting all neighbors // note that the south is +1 becasue array direction +y is going down 
    let neighbor_e = G[current_node.i+1][current_node.j];
    let neighbor_s = G[current_node.i][current_node.j+1];
    let neighbor_w = G[current_node.i-1][current_node.j];
    let neighbor_n = G[current_node.i][current_node.j-1];

    // maps a char to a neighbor
    const order_map = new Map([
        ["n", neighbor_n],
        ["e", neighbor_e],
        ["s", neighbor_s],
        ["w", neighbor_w]
    ]);

    // add neighbor to neighbor vector, use the order of the char in the order string to determine the order
    for(let i = 0; i < order.length; i++){
        neighbors.push(order_map.get(order.charAt(i)));
    }
    
    return neighbors;
}

function addValidNeighbors(current_node, neighbors){
    if (search_alg == "depth-first" || search_alg == "breadth-first"){
        // for all neighbors
        for (let i = 0; i < neighbors.length; i++){
            // if it has not been visited, and it is not in collision, and it is not already queued
            if((!(neighbors[i].visited)) && !(testCollision([neighbors[i].x,neighbors[i].y])) &&(!(neighbors[i].queued))){ 
                //add it to queue 
                visit_queue.push(neighbors[i]);
                neighbors[i].queued = true;

                // get distance from current node to neighbor
                let distance_2_neighbor = Math.sqrt(((current_node.x + neighbors[i].x)**2) + ((current_node.y + neighbors[i].y)**2)); 

                // update each neighbors distance from the start location
                // note that the initial distance is initialized as infinity. if the current distance is greater than the new distance of thee newly found neighbor then update it 
                if(neighbors[i].distance > current_node.distance + distance_2_neighbor){ 
                    neighbors[i].distance = current_node.distance + distance_2_neighbor;
                    neighbors[i].parent = current_node; 
                }

                // draw the symbol for a queued node 
                draw_2D_configuration([neighbors[i].x, neighbors[i].y], "queued");
            }
        } 
    } 
    else if(search_alg == "A-star"){
        for (let i = 0; i < neighbors.length; i++){
            // check if it meets criteria
            if((!(neighbors[i].visited)) && !(testCollision([neighbors[i].x,neighbors[i].y])) && (!(neighbors[i].queued))){ 
    
                // get the distance from current node to the neighbor
                let distance_2_neighbor = Math.sqrt(((current_node.x - neighbors[i].x)**2) + ((current_node.y - neighbors[i].y)**2));
                
                if(neighbors[i].distance > current_node.distance + distance_2_neighbor){
                    // set parent node
                    neighbors[i].parent = current_node;
    
                    // update the neighbor distance = g_score = distance from neighbor back to start along current path
                    neighbors[i].distance = current_node.distance + distance_2_neighbor; 
    
                    // determine distance from the neighbor node to the goal node. called h_score
                    let h_score = Math.sqrt(((neighbors[i].x - goal_node.x)**2) + ((neighbors[i].y - goal_node.y)**2)); // note ** is exponentiation // h_score... // distance from neighbor node to goal node
                    
                    // calculate f_score and assign it. fscore = current distance from start to the neighbor + h score
                    neighbors[i].priority = neighbors[i].distance + h_score; 
    
                    // add to the visit queue and perform min heap insertion
                    minheap_insert(neighbors[i], visit_queue);
                    
                    //set queued paramater to true and draw it on the map
                    neighbors[i].queued = true;
                    draw_2D_configuration([neighbors[i].x, neighbors[i].y], "queued");
                }
           }
        }
    }
    else if(search_alg == "greedy-best-first"){
        for (let i = 0; i < neighbors.length; i++){
            // check if it meets criteria
            if((!(neighbors[i].visited)) && !(testCollision([neighbors[i].x,neighbors[i].y])) && (!(neighbors[i].queued))){
                
                // get the distance from current node to the neighbor
                distance_2_neighbor = Math.sqrt(((current_node.x - neighbors[i].x)**2) + ((current_node.y - neighbors[i].y)**2));
                
                if(neighbors[i].distance > current_node.distance + distance_2_neighbor){
                    // set parent
                    neighbors[i].parent = current_node;

                    // g_score = distance from neighbor back to start along current path
                    neighbors[i].distance = current_node.distance + distance_2_neighbor; 

                    // h_score = distance from neighbor to the goal node
                    let h_score = Math.sqrt(((neighbors[i].x - goal_node.x)**2) + ((neighbors[i].y - goal_node.y)**2));
                    
                    // f_score
                    neighbors[i].priority = h_score;

                    // add to the visit queue, draw it, and perform min heap insertion
                    neighbors[i].queued = true;
                    minheap_insert(neighbors[i], visit_queue); 
                    draw_2D_configuration([neighbors[i].x,neighbors[i].y], "queued");
                }            
            }
        }
    }
}

function checkGridSquare(q, min_x, max_x, min_y, max_y){
    /*
        Determine if the given coordinate is in the bounds of the grid square and return true or false
        Note: left and top boundary of grid square is included in grid, right and bottom boundary is excluded
        Input parameters:
            q: (2-element 1D array with coordinates) given coordinate to check
            min_x: (float) minimum x boundary 
            max_x: (float) maximum x boundary
            min_y: (float) minimum y boundary 
            max_y: (float) maximum y boundary 
    */

    if((q[0] >= min_x) && (q[0] < max_x) && (q[1] >= min_y) && (q[1] < max_y)){ 
        return true;
    }
    else{ 
        return false;
    }


}

function A_star(order = "ewsn"){
    // get the highest priority node wrt f_score 
    let current_node = minheap_extract(visit_queue);
        
    // set current node to visited, draw it on map, iterate number of nodes visited
    current_node.visited = true;
    draw_2D_configuration([current_node.x,current_node.y], "visited")
    search_visited += 1;

    //check if the current node is the solution
    if((current_node.i == goal_index_x) && (current_node.j == goal_index_y)){
        search_iterate = false;
        drawHighlightedPathGraph(current_node); 
        return "succeeded";
    }
    
    // find the neighbors
    let neighbors = findNeighbors(current_node, order); 
    
    // adds valid neighbors to the visit_queue
    addValidNeighbors(current_node, neighbors)

    // check if there are any nodes in the queue
    if (visit_queue.length == 0){ 
        search_iterate = false;
        drawHighlightedPathGraph(current_node); 
        return "failed";
    }

    return "iterating"; // default  
}

function dfs(order = "ewsn"){
    /*
    order: (string) optional  parameter to specify the order that the neighbors are discovered in. it is passed to find neighbors function
    Use letters n,e,s,w in a string to specify the order, default order is East, West, South, North
    */
    
    // get the most recently added element: stack method = LIFO
    let current_node = visit_queue.pop();
        
    // setting current node to visited, drawing it on map, iterate number of nodes visited
    current_node.visited = true;
    draw_2D_configuration([current_node.x,current_node.y], "visited")
    search_visited += 1;

    //check if the current node is the solution
    if((current_node.i == goal_index_x) && (current_node.j == goal_index_y)){
        search_iterate = false;
        drawHighlightedPathGraph(current_node); 
        return "succeeded";
    }
    // find the neighbors
    neighbors = findNeighbors(current_node, order);
   
    // adds valid neighbors to the visit_queue also has a function in it to edit the params of each neighbor that is added
    addValidNeighbors(current_node, neighbors)

    if (visit_queue.length == 0){
        search_iterate = false;
        drawHighlightedPathGraph(current_node);    
        return "failed";
    }
    
    return "iterating"; // default  
}

function bfs(order = "ewsn"){
    // get the first element: queue method = FIFO  
    let current_node = visit_queue.shift();

    // set current node to visited, draw it on map, iterate number of nodes visited
    current_node.visited = true;
    draw_2D_configuration([current_node.x,current_node.y], "visited")
    search_visited += 1;

    //check if the current node is the solution
    if((current_node.i == goal_index_x) && (current_node.j == goal_index_y)){
        search_iterate = false;
        drawHighlightedPathGraph(current_node); 
        return "succeeded";
    }

    // find the neighbors
    let neighbors = findNeighbors(current_node, order); 

    // adds valid neighbors to the visit_queue also has a function to edit the params of each neighbor that is added
    addValidNeighbors(current_node, neighbors)

    if (visit_queue.length == 0){
        search_iterate = false;
        drawHighlightedPathGraph([current_node.x,current_node.y]);    
        return "failed";
    }
    
    return "iterating"; // default 

}

function greedy_best_first_search(order = "ewsn"){
    // get the highest priority node wrt f_score 
    let current_node = minheap_extract(visit_queue); 
        
    // set current node to visited, draw it on map, iterate number of nodes visited
    current_node.visited = true;
    draw_2D_configuration([current_node.x,current_node.y], "visited")
    search_visited += 1;

    // checking if current node is the solution
    if((current_node.i == goal_index_x) && (current_node.j == goal_index_y)){
        search_iterate = false;
        drawHighlightedPathGraph(current_node); 
        return "succeeded";
    }

    // find the neighbors
    let neighbors = findNeighbors(current_node, order); // can change order of neighbors in this function
   
    // adds valid neighbors to the visit_queue also has a function to edit the params of each neighbor that is added
    addValidNeighbors(current_node, neighbors)

    if (visit_queue.length == 0){
        search_iterate = false;
        drawHighlightedPathGraph([current_node.x,current_node.y]);    
        return "failed";
    }
    
    return "iterating"; // default 
}
    
