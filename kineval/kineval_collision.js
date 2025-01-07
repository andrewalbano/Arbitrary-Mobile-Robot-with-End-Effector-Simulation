
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles
    
    // For debugging the collision 
    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    
    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;

    if (robot.collision != false){
        console.log("collision")
        console.log(robot.collision)
        
    }

}

// note that this uses origin of robot to check the boundary collision for world boundary
kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;

    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
     
    return robot_collision_forward_kinematics(q)
}

function robot_collision_forward_kinematics(q){
    // generate the start of the stack
    let mstack = generate_identity()

    // begin building the stack. returns the stack after adjusting for the global origin
    mstack = buildFKTransforms_collision(q, mstack)

    let link = robot.links[robot.base]
    
    let in_collision = traverse_collision_forward_kinematics_link(link,mstack,q) 
    return in_collision
    
}

function traverse_collision_forward_kinematics_joint(joint, mstack, q){
    // getting the name of the joint as a string
    let j  = q_names[joint.name]

    //compute local transform from the translation and rotations
    let DL = generate_translation_matrix(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2])
    let Rx = generate_rotation_matrix_X(joint.origin.rpy[0])
    let Ry = generate_rotation_matrix_Y(joint.origin.rpy[1])
    let Rz = generate_rotation_matrix_Z(joint.origin.rpy[2])
        
    let RL = matrix_multiply(Rz,Ry)
    RL = matrix_multiply(RL,Rx)
    let local_transform = matrix_multiply(DL,RL)
        
    mstack  = matrix_multiply(mstack,local_transform) //update mstack to get new transform
 
    // accounting for motors at the joints
    if( joint.type == "prismatic"){
        let unitJointAxis = vector_normalize(joint.axis)

        // generating translation matrix in direction of unit joint axis
        let motorTransform = generate_translation_matrix(unitJointAxis[0], unitJointAxis[1], unitJointAxis[2]) 
        
        //scaling the translation using the angle... (angle in this sense refers to the amount of translation, think of product of exponential definition for theta)
        motorTransform[0][3] *= q[j]
        motorTransform[1][3] *= q[j]
        motorTransform[2][3] *= q[j]

        //getting the new value for the stack 
        mstack = matrix_multiply(mstack,motorTransform)
        
    }
    else if( joint.type == "continuous"){ //using quarternions
        let m = kineval.quaternionFromAxisAngle(joint.axis,q[j])
        m = kineval.quaternionNormalize(m)
        let motorTransform = kineval.quaternionToRotationMatrix(m)
        mstack = matrix_multiply(mstack,motorTransform)
    }
    else if( joint.type == "revolute"){ //using quarternions
        let m = kineval.quaternionFromAxisAngle(joint.axis, q[j])
        m = kineval.quaternionNormalize(m)
        let motorTransform = kineval.quaternionToRotationMatrix(m)
        mstack= matrix_multiply(mstack,motorTransform)
    }
    else if(joint.type== "fixed"){
        mstack = mstack
    }
    else{ // assume revolute when not specified
        let m = kineval.quaternionFromAxisAngle(joint.axis, q[j])
        m = kineval.quaternionNormalize(m)
        let motorTransform = kineval.quaternionToRotationMatrix(m)
        mstack = matrix_multiply(mstack,motorTransform)
    }
    
    
    //recurse to child link 
    let link = robot.links[joint.child]
    return traverse_collision_forward_kinematics_link(link,mstack,q)
    
}


function buildFKTransforms_collision(q,mstack){
    //world frame, root of kinematic tree

    //computing transform of global wrt to world 
    let DW = generate_translation_matrix(q[0], q[1], q[2])
    let Rx = generate_rotation_matrix_X(q[3])
    let Ry = generate_rotation_matrix_Y(q[4])
    let Rz = generate_rotation_matrix_Z(q[5])
    
        
    let RW = matrix_multiply(Rz,Ry)
    RW = matrix_multiply(RW,Rx)

    let local_transform = matrix_multiply(DW,RW)

    mstack = matrix_multiply(mstack, local_transform) //updating transform

    // robot heading and lateral... not sure if i need the part about robot heading and lateral because im specifying a robot config 
    // for imported geometry change coordinate
    if(robot.links_geom_imported){
        // system transform
        let Rx2 = generate_rotation_matrix_X(-Math.PI/2)
        let Ry2 = generate_rotation_matrix_Y(-Math.PI/2)
        let Rz2 = generate_rotation_matrix_Z(0)
        
        let system_conversion = matrix_multiply(Ry2,Rx2)
        mstack = matrix_multiply(mstack, system_conversion) 
        

        //robot heading and lateral
        let xaxis = [[1],[0],[0],[1]]
        let yaxis = [[0],[-1],[0],[1]] // negative sign accounts for the code given in user input 
       
        // robot_lateral = matrix_multiply(mstack,yaxis)//used for left right... // my xaxis is their y axis 
        // robot_heading = matrix_multiply(mstack,xaxis) //used for forward back // my zaxis is theie x axis
        
    }
    else{
        //robot heading and lateral
        //in our coordinate system.. // these are used for robot lateral and heading
        let xaxis = [[-1],[0],[0],[1]] // negative sign accounts for the code given in user input
        let zaxis = [[0],[0],[1],[1]]
        // robot_lateral= matrix_multiply(mstack,xaxis) //used for left right
        // robot_heading = matrix_multiply(mstack,zaxis)//used for forward back
    }

    return mstack


}


function traverseFKBase_collision(q,mstack){
    
    robot.links[robot.base].xform_config = mstack// transform of link wrt. world
    return mstack

}

function traverse_collision_forward_kinematics_link(link,mstack,q) {

    if (typeof link.visual !== 'undefined') {
        var local_link_xform = matrix_multiply(mstack,generate_translation_matrix(link.visual.origin.xyz[0],link.visual.origin.xyz[1],link.visual.origin.xyz[2]));
    }
    else {
        var local_link_xform = matrix_multiply(mstack,generate_identity());
    }

    // test collision by transforming obstacles in world to link space
    mstack_inv = matrix_invert_affine(mstack);

    //mstack_inv = numeric.inv(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision){
            //console.log("collision detected with link:")//, link.name)

            return link.name;
        }
    }
    
    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q)
            //local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],q)
        
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children 
    return false;
}
