
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {
    
    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];
    
    //q_start_config = initialRandomConfig()

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    //initializing the start and goal trees
    T_a = tree_init(q_start_config)
    T_a.name = "T_a"
    T_b = tree_init(q_goal_config)
    T_b.name = "T_b"
    step_length = 0.2
    search_index = 0 

}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)


    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations

        let q_rand = randomConfig()
    
        // if it is not trapped, try connecting it to the other tree
        if (extendRRT(T_a, q_rand)!="Trapped"){

            // get new coordinate distance eps from nearest neighbor in direction of q_new
            let q_new = T_a.vertices[T_a.newest].vertex


            if(connectRRT(T_b, q_new) == "Reached"){

                // goal is found 

                // stop iterating the search
                rrt_iterate = false

                // get the path from T_a and T_b and connect them 
                let path1 = dfsPath(T_a)
                let path2 = dfsPath(T_b)
                finalPath = combinePaths(path1,path2)
                
                // push the plan to global variable
                for(i = 0; i < finalPath.vertices.length; i++){
                    
                    kineval.motion_plan.push(finalPath.vertices[i])
                }
        
                // change the color of the cells in the path plan
                highlightPath(finalPath)
            
                return "reached"

            }

            else{

                return "extended"

            }
        }

        else{

            [T_a,T_b] = [T_b,T_a]  // swap when trapped

            rrt_iter_count +=1 
            return "extended"
        }     
        
    }
    
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;
    

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

   // get random point on map
function randomConfig(){

    let max_x = robot_boundary[1][0]
    let min_x = robot_boundary[0][0]
    let max_z = robot_boundary[1][2]
    let min_z = robot_boundary[0][2]

    let min_pitch = -Math.PI
    let max_pitch = Math.PI
    
    let rand_x = (Math.random() * (max_x- min_x) + min_x)//.toFixed(3);
    let rand_z= (Math.random() * (max_z- min_z) + min_z)//.toFixed(3);
    let rand_pitch = (Math.random() * (max_pitch - min_pitch) + min_pitch)//.toFixed(3)
    
    let q_rand = [rand_x,0,rand_z,0,rand_pitch,0] //y is always zero

    for (x in robot.joints) {
     
        rand_pitch = (Math.random() * (max_pitch - min_pitch) + min_pitch)
        q_rand = q_rand.concat(rand_pitch);
    }
    
    return q_rand

    
}

function extendRRT(tree,q){
    let q_nearest_info = nearestNeighbor(tree,q)
    let q_nearest = q_nearest_info[0]
    let q_nearest_index = q_nearest_info[1]

    let q_new = findQNew(q_nearest,q)
    
    if(kineval.poseIsCollision(q_new) == false){
        
        // insert vertex
        tree_add_vertex(tree, q_new)

        //add parent index, and current index for debugging and tracing path 
        tree.vertices[tree.newest].parent = q_nearest_index //using this to trace pathway back when complete
        tree.vertices[tree.newest].index = tree.newest
    
        // insert edge
        tree_add_edge(tree, q_nearest_index, tree.newest) 
       
        // check distance in XZ plane to the target q in XZ plane
        let dist = distance([q_new[0],q_new[2]], [q[0],q[2]])
                    
        if (dist < step_length/2){ // if they are within the tolerance. within ball of epsilon such that no collision because within the same square 
            //add the target q vertex to the tree
            tree_add_vertex(tree, q)
            tree_add_edge(tree, tree.newest-1, tree.newest) 
            tree.vertices[tree.newest].parent = tree.newest-1 
            tree.vertices[tree.newest].index = tree.newest
            return "Reached"
        }
        else{
            return "Advanced"
        }

    }
    return "Trapped"
   
}

function connectRRT(tree, q){
    let S = "Advanced" // initialize  S as advanced 
    
    while (S == "Advanced"){
        S = extendRRT(tree,q)           
    }

    return S

}

function nearestNeighbor(tree, q){
    let nearest_neighbor
    let nearest_neighbor_distance = 1000000000000000 // initializing to infinity
    let nearest_neighbor_index = -1 //default

    for (let i = 0; i < tree.vertices.length; i++){
        // checking distance to each neighbor 
        let neighbor_distance = distance([q[0],q[2]], [tree.vertices[i].vertex[0],tree.vertices[i].vertex[2]])

        // reassigning values for nearest neighbor if a close neighbor is found
        if (neighbor_distance < nearest_neighbor_distance){
            nearest_neighbor = tree.vertices[i].vertex
            nearest_neighbor_distance = neighbor_distance
            nearest_neighbor_index = i
        }
    }  
    return [nearest_neighbor,nearest_neighbor_index]
    
}

function findQNew(q_near, q){

    //map joints in q_names to indexs
    let joint_name = [0,0,0,0,0,0]

    for (x in robot.joints) {
        //joint_index.push(q_names[x])
        joint_name.push(robot.joints[x].name)
    }
    // get delta in x and z from nearest point to target point 
    let delta_x = q[0] - q_near[0]
    let delta_z = q[2] - q_near[2]

    // get direction to step in
    let theta = Math.atan2(delta_z, delta_x)
    let step_length_rad = 0.5

    // get new q
    let q_new = [q_near[0] + (step_length * Math.cos(theta)), 0, q_near[2] + (step_length * Math.sin(theta))]
    // getting vector from nearest config to target config
    let q_joints = [0,0,0]
    
    for(let i = 3; i<q.length; i++){
        
        if (i == 3 || i == 5){ // just added this 
            q_joints.push(0)
        }
        else{
            q_joints.push(q[i] - q_near[i])
        }
    }

    //normalizing the vector
    let sum = 0

    for(let i = 0; i<q_joints.length; i++){
        sum = sum + (q_joints[i]*q_joints[i])
    }

    let magnitude = Math.sqrt(sum)

    //adding the new joint states
    for(let i = 3; i<q.length; i++){
        if (i == 3 || i == 5){//just added this dont want to flip the robot around these axis
            q_new.push(0)
        }
        else{
            
            let angle = step_length_rad*(q_near[i] + (q_joints[i]/magnitude))

            // enforcing jpoint limits
            if (i>5 && robot.links_geom_imported){

                if(robot.joints[joint_name[i]].type == "revolute" || robot.joints[joint_name[i]].type == "prismatic"){
                    if (angle > robot.joints[joint_name[i]].limit.upper){
                        q_new.push(robot.joints[joint_name[i]].limit.upper)
                    }
                    else if (angle < robot.joints[joint_name[i]].limit.lower){
                        q_new.push(robot.joints[joint_name[i]].limit.lower)
                    } 
                    else {
                        q_new.push(angle)
                    }
                }
                else {
                    q_new.push(angle)
                }
                
            }
            else{
                q_new.push(angle)
            }
        }
    }

    return q_new
}


function distance(q1,q2){ 
    return Math.sqrt(((q1[0]-q2[0])**2) + ((q1[1] - q2[1])**2))
}

function dfsPath(tree){
    let index = tree.newest
    let path = tree_init(tree.vertices[index].vertex)
    path.name = tree.name
    
    while(true){ 
        let parent_index = tree.vertices[index].parent
        path.vertices.push(tree.vertices[parent_index])
        index = parent_index
        if (parent_index == 0){
            break
        }

    }
    return path

}

// combines paths from T_a and T_b for display 
function combinePaths(tree1,tree2){
    
    let index1 = tree1.vertices.length-1
    let index2 = tree2.vertices.length-1

    let path = tree_init(q_start_config)

    if (tree1.name == "T_a"){
        
        for(i = index1-1; i >= 0; i--){
            path.vertices.push(tree1.vertices[i])
        }

        for(i = 1; i <tree2.vertices.length; i++){
            path.vertices.push(tree2.vertices[i])
        }
    }
    else{
            
        for(i = index2-1; i >=0; i--){
            path.vertices.push(tree2.vertices[i])
        }

        for(i = 1; i < tree1.vertices.length; i++){
            path.vertices.push(tree1.vertices[i])
        }
        
    }

     
    return path
}


function highlightPath(tree){
    for (let i = 0; i<tree.vertices.length; i++){
        tree.vertices[i].geom.material.color = {r:1,g:0,b:0};
    }
}


function normalize_joint_state(q){
    for(let i = 0; i<q; i++){

    }

}