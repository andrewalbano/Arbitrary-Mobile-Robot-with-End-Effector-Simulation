/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

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

ballRadius = 0.5 // variable for radius of nodes to search for parent in 
search_index = 0 

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

    // get random location, added some bias so that it works toward the goal more efficiently 
    q_rand = randomConfig()
    
    let state = extendRRT(T_a, q_rand)

    if(state == "Reached"){
        search_iterate = false
        let path = dfsPath(T_a)
        drawHighlightedPath(path.vertices)
        return "succeeded"
    }
    else if(search_index > search_max_iterations){
        return "failed"
    }

    else if(state == "Advanced" || state == "Trapped"){
        search_index += 1
        return "extended"
    }
}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
    
    // get random coordinate
    q_rand = randomConfig()

    // if it is not trapped, try connecting it to the other tree
    if (extendRRTconnect(T_a, q_rand)!="Trapped"){

        // get new coordinate distance eps from nearest neighbor in direction of q_new
        let q_new = T_a.vertices[T_a.newest].vertex


        if(connectRRT(T_b, q_new) == "Reached"){

            // goal is found 
            search_iterate = false
            
            // get the path from T_a and T_b and connect them 
            let path1 = dfsPath(T_a)
            let path2 = dfsPath(T_b)
            let finalPath = combinePaths(path1,path2)
            drawHighlightedPath(finalPath.vertices)
            return "succeeded"

        }
        else if(search_index > search_max_iterations){
            search_index = 0 
            return "failed"
        }
        else{
            search_index = 0
            return "extended"

        }
    }

    else{
        search_index = 0
        [T_a,T_b] = [T_b,T_a]  // swap when trapped

        return "extended"
    }     

}

function iterateRRTStar() {
  
    // get random point added some bias towards the goal location
    q_rand = randomConfig()
    
    // extend RRT
    let state = extendRRTstar(T_a, q_rand)
    
    if(state == "Reached"){
        search_iterate = false
        let path = dfsPath(T_a)
        drawHighlightedPath(path.vertices)
        return "succeeded"
    }
    else if(search_index > search_max_iterations){
        return "failed"
    }
    else if(state == "Advanced" || state == "Trapped"){
        search_index = 0
        return "extended"
    }
    else{
        search_iterate = false
        return "failed"
    }
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath

    // Extend functions
    function extendRRT(tree,q){
        
        // find the nearest neighhbor and its index in the tree
        let [q_nearest, q_nearest_index] = findNearestNeighbor(q,tree)
        
        // get the new point
        let q_new = findQNew(q_nearest,q)

        //check if the new configuration is collision free
        if (newConfig(q_new)){

            // get distance between new point and nearest point
            let dist = Distance(q_new, q_nearest)
            
            // add the new q to the tree
            insertTreeVertex(tree,q_new)

            // set the parent node for the newest value in the tree as teh nearest neighbor and set the newest index 
            tree.vertices[tree.newest].parent = q_nearest_index // using this to trace pathway back when complete
            tree.vertices[tree.newest].index = tree.vertices.length-1 // using this to trace pathway back when complete
            
            //set the cost of the first value in the tree equal to zero
            if (q_nearest_index == 0){
                tree.vertices[0].cost = 0
            }
            // get the cost of the newest q 
            tree.vertices[tree.newest].cost = tree.vertices[q_nearest_index].cost + dist 
            
            // insert the tree edge
            insertTreeEdge(tree, q_nearest_index, tree.newest)  

            // get the distance from q_new to the goal  
            let dist2goal = Distance(q_new,q_goal)
            if (dist2goal <  eps){ //q_new is approximately q // if they are within the tolerance. within ball of epsilon such that no collision because within the same square            
                //add the target q vertex to the tree
                insertTreeVertex(tree,q_goal)
                insertTreeEdge(tree, tree.newest-1, tree.newest)
                tree.vertices[tree.newest].parent = tree.newest-1 //using this to trace pathway back when complete
                tree.vertices[tree.newest].index = tree.newest //using this to trace pathway back when complete
                tree.vertices[tree.newest].cost = tree.vertices[tree.newest-1].cost + dist2goal 
                return "Reached"
            }
            else{
                return "Advanced"
            }
        }
        // if there is a collision return trapped 
        return "Trapped" 
        
        
        
    }
    
    function extendRRTconnect(tree,q){
        // find the nearest neighhbor and its index in the tree
        let [q_nearest, q_nearest_index] = findNearestNeighbor(q,tree)

        // get the new point
        let q_new = findQNew(q_nearest,q)
        
   
        if (newConfig(q_new)){ // if no collision 
        
            insertTreeVertex(tree,q_new)
            tree.vertices[tree.newest].parent = q_nearest_index //using this to trace pathway back when complete
            //tree.vertices[tree.newest].index = tree.vertices.length-1 //using this to trace pathway back when complete

            insertTreeEdge(tree, q_nearest_index, tree.newest)  

            // check distance to the target q
            let dist = Distance(q_new, q)
                        
            if (dist < eps){ // if they are within the tolerance. within ball of epsilon such that no collision because within the same square 
                //add the target q vertex to the tree
                insertTreeVertex(tree,q)
                insertTreeEdge(tree, tree.newest-1, tree.newest)
                tree.vertices[tree.newest].parent = tree.newest-1 
                return "Reached"
            }
            else{
                return "Advanced"
            }
        }
        return "Trapped" 
    }

    
    function extendRRTstar(tree,q){
        // get nearest neighbor information. Need the vertex and the index in T_a
        let [q_nearest, q_nearest_index] = findNearestNeighbor(q,tree) 

        // find the new point. step size epsilon in direction of q_new from the nearest point
        let q_new = findQNew(q_nearest,q) //gets the new config
    
        if (newConfig(q_new)){ // if new config is a viable configuration... == if there is no collision 

            // get the index in T_a for all vertices within ball radius
            let q_near_index_list = nearPoints(tree, q_new, q_nearest_index)
           
            let parent_index = q_nearest_index // By default use this 

            // Determine the best parent 
            if (q_near_index_list.length >1){
                parent_index = chooseParent(tree, q_near_index_list, q_nearest_index, q_new)
            }
            
            // add the vertex and edge to the tree
            insertTreeVertex(tree,q_new)
            insertTreeEdge(tree, parent_index, tree.newest)

            // extra parameters
            tree.vertices[tree.newest].parent = parent_index//using this to trace pathway back when complete
            tree.vertices[tree.newest].index = tree.newest //using this to trace pathway back when complete
            tree.vertices[tree.newest].cost =  tree.vertices[parent_index].cost + Distance(tree.vertices[parent_index].vertex, q_new) //tree.vertices[parent_index].cost + dist to q_new
            
            //rewire function
            rewire(tree, q_near_index_list)
            
            // check if the goal is reached
            dist2goal = Distance(tree.vertices[tree.newest].vertex, q_goal)
            if (dist2goal <  eps){ // if they are within the tolerance. within ball of epsilon such that no collision because within the same square 
                //add the target q vertex to the tree
                insertTreeVertex(tree,q_goal)
                insertTreeEdge(tree, tree.newest-1, tree.newest)
                tree.vertices[tree.newest].parent = tree.newest-1 //using this to trace pathway back when complete
                tree.vertices[tree.newest].index = tree.newest //using this to trace pathway back when complete
                tree.vertices[tree.newest].cost = tree.vertices[tree.newest-1].cost + dist2goal 
                return "Reached"
            }
            else{
                return "Advanced"
            }
        }
        return "Trapped" 
        
    }
    


    // RRT Connect helper function 

    // attempt to create pathway from one tree to the other 
    function connectRRT(tree, q){
        let S = "Advanced" // initialize  S as advanced 
        search_index = 0
        while (S == "Advanced"){
            S = extendRRTconnect(tree,q)     
            search_index +=1
            if(search_index > search_max_iterations){
                return "failed"
            }       
        }

        return S

    }

    // combines paths from T_a and T_b for display 
    function combinePaths(tree1,tree2){
       
        let index1 = tree1.vertices.length-1
        let index2 = tree2.vertices.length-1

        let path = initRRT(q_goal)
        let new_vertex = {};
        new_vertex.edges = [];

        // of start is in tree 1 and goal is in tree 2
        if (tree1.vertices[index1].vertex[0] == q_init[0] && tree1.vertices[index1].vertex[1] == q_init[1]){
                
            for(i = index2-1; i >= 0; i--){
                path.vertices.push(tree2.vertices[i])
            }
    
            for(i = 0; i <= index1; i++){
                path.vertices.push(tree1.vertices[i])
            }
            
            
        }
        // start is in tree 2 and goal is in tree 1
        else if(tree2.vertices[index2].vertex[0] == q_init[0] && tree2.vertices[index2].vertex[1] == q_init[1]){ 
            
            for(i = index1-1; i >=0; i--){
                path.vertices.push(tree1.vertices[i])
            }
    
            for(i = 0; i <= index2; i++){
                path.vertices.push(tree2.vertices[i])
            }
            
        }

        return path
    }

    

    //RRT star helper functions

    // returns the index of all points in the tree that are within ball radius
    function nearPoints(tree, q_new, q_near_index){
        // get the index of all vertices within the ball radius
        q_near_index = []
        // check all point in the tree to see if they are within ball radius of q_new
        for (let i = 0; i<tree.vertices.length; i++){

            if (Distance(tree.vertices[i].vertex, q_new) < ballRadius){ // if within ball, add the index to the list
                q_near_index.push(i)
            }
        }
        return q_near_index
        /* 
            let points = initRRT(tree.vertices[q_near_index].vertex)
            //let parent_index = tree1.vertices[index].parent
            //path1.push(parent_index)
            
            let new_vertex = {};
            new_vertex.edges = [];
            
            for(let i=0; i<tree.vertices.length; i++){ // check if within ball
                if( i!=q_near_index ){
                    let dist = Distance(q_new,tree.vertices[i].vertex) 
                    //console.log("ballRadius ", ballRadius, "distance,", dist)
                    if(dist < ballRadius){
                        
                        points.vertices.push(tree.vertices[i])//.vertex)
                        points.vertices[points.newest].index = i // index in T_a
                        //console.log("it is within tolerance")
                        
                    }
                }
                    
            }
            return points
        */
    }

    // determines the parent with the least cost path to the new point
    function chooseParent(tree, q_near_index_list, q_min_index ,q_new){
        
        // get the default minCost using q_nearest
        let minCost = tree.vertices[q_min_index].cost + Distance(tree.vertices[q_min_index].vertex, q_new) // = cost to nearest point + cost to reach q_new

        // determine if there is a lower cost path to q_new using the nearby points
        for(let i = 0; i < q_near_index_list.length; i++){
            q_near_index = q_near_index_list[i] 
            // need to check all points on path to q_new

            q_new_prime = findQNew(tree.vertices[q_near_index].vertex, q_new) 

                 
            // iterativaly checking all points on the path to q_new from the q_near node, if it can be reached then there is a collision free path from the nearby node
            // this is most impsactful around corners 
            let state = connect_q_new(tree.vertices[q_near_index].vertex, q_new)

            if(state == "Reached"){

                if(newConfig(q_new_prime)){ // if the new point is a viable point...no collision
                    cost_prime = tree.vertices[q_near_index].cost + Distance(q_new_prime, q_new) 
                    // = cost to current nearby point + cost from the qnew prime to the target q_new ... 
                    // note that this doesnt include the cost from the nearby point to q_new_prime
                
                    // update the parent if there is a better path
                    if (cost_prime < minCost){

                        minCost = cost_prime
                        q_min_index = q_near_index
                    }
                }
            }
        }
        
        return q_min_index
    }

    // checks all points within ball radius of new point and sees if making the new point the parent results in a better cost
    function rewire(tree, q_near_index_list){
        //checking cost to each point in the the ball radius using q_new and rewiring if necessart
        for(let i = 0; i < q_near_index_list.length; i++){

            q_near_index = q_near_index_list[i]

            // need to make sure there are no collisions on new path
            let state = connect_q_new(tree.vertices[tree.newest].vertex, tree.vertices[q_near_index].vertex)

            //if there were no collisions
            if(state == "Reached"){

                let cost = tree.vertices[tree.newest].cost + Distance(tree.vertices[q_near_index].vertex, tree.vertices[tree.newest].vertex)
                let current_cost  = tree.vertices[q_near_index].cost
                
                if(cost < current_cost){
                    // updating the last in the nearby point to use the new vertex as the parent
                    tree.vertices[q_near_index].edges[0] = tree.vertices[tree.newest]
                    tree.vertices[q_near_index].parent = tree.newest
                    tree.vertices[q_near_index].cost = cost
                    draw_2D_edge_configurations(tree.vertices[tree.newest].vertex,tree.vertices[q_near_index].vertex);
                }       
            }
        }
             
    }

    // Conncect_q_new and checkTrajector iterate by distance epsilon the pathway from a point to another point and returns if the path is collision free
    function connect_q_new(q_near, q_new){
        // same concept as connectRRT function
        let S = "Advanced"
        search_index = 0
        while(S == "Advanced"){
            info = checkTrajectory(q_near, q_new)
            q_near = info[0]
            S = info[1]
            search_index +=1
            if(search_index > search_max_iterations){
                return "failed"
            }
        }
        return S
    }

    function checkTrajectory(q_near, q_target){
        let q_new = findQNew(q_near,q_target)

        if(newConfig(q_new)){ // if there is no collision

            //check if qnew is reached
            let dist = Distance(q_new, q_target)
            if (dist < eps){
                return [q_new, "Reached"]
            }
            else{
                return [q_new,"Advanced"]
            }
        }
        else{
            return [q_new,"Failed"]
        }
    }


    // General helper functions

    // get random point on map
    function randomConfig(){
        /*
        canvas coords 800,800
        world coords 7,7
        */
        let max_x = 7
        let min_x = -1
        let max_y = 7
        let min_y = -1

        //rounding to 3 decimals
        let rand_x = (Math.random() * (max_x- min_x) + min_x)//.toFixed(3);
        let rand_y= (Math.random() * (max_y- min_y) + min_y)//.toFixed(3);
        let q_rand = [rand_x,rand_y]

        // adding some bias towards goal
        if(search_iter_count%100 == 0 && (search_alg == "RRT-star" || search_alg =="RRT")){
            q_rand = q_goal
        }
       
        return q_rand

    }

    // find the nearest neighbor to q
    function findNearestNeighbor(q,tree){
        let nearest_neighbor
        let nearest_neighbor_distance = 1000000000000000 // initializing to infinity
        let nearest_neighbor_index = -1
        for (let i = 0; i < tree.vertices.length; i++){
            // checking distance to each neighbor 
            let neighbor_distance = Distance(q, tree.vertices[i].vertex)

            // reassigning values for nearest neighbor if a close neighbor is found
            if (neighbor_distance < nearest_neighbor_distance){
                nearest_neighbor = tree.vertices[i].vertex
                nearest_neighbor_distance = neighbor_distance
                nearest_neighbor_index = i
            }
        }  
        return [nearest_neighbor,nearest_neighbor_index]
    }

    // find a new point q_new in direction of q with distance epsilon from q_near
    function findQNew(q_near, q){

        // get delta in x and y from nearest point to target point 
        let delta_x = q[0] - q_near[0]
        let delta_y = q[1] - q_near[1]
    
        // get direction to step in
        let theta = Math.atan2(delta_y, delta_x)
        
        // for debugging
        let theta_deg = theta*180/Math.PI

        // finding the new point 
        let q_new = [q_near[0] + (eps * Math.cos(theta)), q_near[1] + (eps * Math.sin(theta))]
        return q_new
    }

    // check if the new q is a viable point ie no collision
    function newConfig(q_new){
        
        //testCollision(q_new)
        if (testCollision(q_new)== false){
            return true
        }
        else{
            return false
        }
        
        
    }

    // returns distance from two points
    function Distance(q1,q2){

        return Math.sqrt(((q1[0]-q2[0])**2) + ((q1[1] - q2[1])**2))
      
    }

    // creates a tree with the pathway using the parents 
    function dfsPath(tree){
        let index = tree.newest
        let path = initRRT(tree.vertices[index].vertex)

        while(true){ 
            let parent_index = tree.vertices[index].parent
            path.vertices.push(tree.vertices[parent_index])
            index = parent_index
            if (parent_index == 0){
                //path.vertices.push(tree.vertices[index]) should i be adding this
                break
            }

        }
        return path

    }

