
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

   // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();
    // STENCIL: see instructor for random time trial code

    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
        + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
        + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
    kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
    kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
    kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
    kineval.params.trial_ik_random.targets += 1;
    textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length


    // get target position and orientation in world frame as 6 by 1 vector
    let ee_target = []
    ee_target[0] = [endeffector_target_world.position[0]]
    ee_target[1] = [endeffector_target_world.position[1]]
    ee_target[2] = [endeffector_target_world.position[2]]
    ee_target[3] = [endeffector_target_world.orientation[0]]
    ee_target[4] = [endeffector_target_world.orientation[1]]
    ee_target[5] = [endeffector_target_world.orientation[2]]


    //location of effector world coordinates
    let ee_location = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local)
    //converting it to 6 by 1, initially assigning the last 3 to be equal to the orientation of the block
    let ee_current = []
    ee_current[0] = [ee_location[0]]
    ee_current[1] = [ee_location[1]]
    ee_current[2] = [ee_location[2]]
    ee_current[3] = [endeffector_target_world.orientation[0]]
    ee_current[4] = [endeffector_target_world.orientation[1]]
    ee_current[5] = [endeffector_target_world.orientation[2]]  

    //if orientation is required get the euler angles
    if (kineval.params.ik_orientation_included){
        //Used Tait-Bryan XYZ order from wikipedia: https://en.wikipedia.org/wiki/Euler_angles
        //calculate euler angles for EE world 
        let beta = Math.asin(robot.joints[endeffector_joint].xform[0][2])
        let alpha = Math.atan2(-robot.joints[endeffector_joint].xform[1][2], robot.joints[endeffector_joint].xform[2][2])
        let gamma = Math.atan2(-robot.joints[endeffector_joint].xform[0][1], robot.joints[endeffector_joint].xform[0][0])
        
        //finish initialize the 6 by 1 vector for current EE location and orientation
        ee_current[3] = [alpha]
        ee_current[4] = [beta]
        ee_current[5] = [gamma]  
    } 

    //calculate the error between target and current
    robot.dx = []
    for(i in ee_current){
        robot.dx[i] = [ee_target[i] - ee_current[i]]

    }   

    //compute chain from base to end effector with an error check so it will only compute chain the first time
    try {
        ik_chain.length
    }
    catch(err) {
        ik_chain = determineChain(robot.base, endeffector_joint) //calculate the chain
        ik_chain.reverse()//reverse the order 

        console.log("chain from end effector to base")
        console.log(ik_chain)
    }
    
    //calculate jacobian with respect to joints along the chain from end effector joint to the robot base
    calculateJacobian(ee_current)
    
    //attempting stochastic gradient descent

    // try {
    //     robot.jacobian.length //ik_chain.length
    //     calculateJacobian_stochastic(ee_current)
    // }
    // catch(err) {
    //     calculateJacobian(ee_current)
    // }
    // //calculateJacobian_stochastic(ee_current)

           
    if (kineval.params.ik_pseudoinverse){
        // computing inverse with pseudo inverse
        let jacobian_inverse = matrix_pseudoinverse(robot.jacobian)
            
        // computing step direction with pseudoinverse
        robot.dq = matrix_multiply(jacobian_inverse, robot.dx)
    
    }
    else{       
        // computing jacobian transpose
        let jacobian_transpose = matrix_transpose(robot.jacobian)
          
        // computing step direction with jacobian transpose
        robot.dq = matrix_multiply(jacobian_transpose,robot.dx)
    }
        

    //update control
    dtheta = []
    for(i in ik_chain){
        dtheta[i] = kineval.params.ik_steplength * robot.dq[i]
        robot.joints[ik_chain[i]].control = dtheta[i]
        }

}

function determineChain(base, end_effector){
    let current_node = {}
    current_node.name = end_effector
    current_node.type = "joint"

    let chain = [current_node.name]

    while(robot.joints[current_node.name].parent != base){ // base holds the name of the base link 
        current_node.name = robot.links[robot.joints[current_node.name].parent].parent
        chain.push(current_node.name)
    }
    
    // alternate method
    // while(current_node.name != base){ // base holds the name of the base link 
        
    //     if(current_node.type == "joint"){
    //         current_node.name = robot.joints[current_node.name].parent
    //         current_node.type = "link"
    //     }
    //     else if (current_node.type == "link"){
    //         current_node.name = robot.links[current_node.name].parent
    //         current_node.type = "joint"
    //         chain.push(current_node.name)
    //     }   
    // }

    return chain
}

function calculateJacobian(ee_current){
    
    //calculate jacobian with respect to joints along the chain from end effector joint to the robot base
    robot.jacobian = generate2D(6,ik_chain.length) //initialize matrix of required size

    // generate values for the jacobian
    for(i in ik_chain){
        //see slide 64        

        // computing global axis of joint i

        //difference between end effector location and joint i origin in world coords
        let term2 = []
        term2[0] = [ee_current[0] - robot.joints[ik_chain[i]].xform[0][3]]
        term2[1] = [ee_current[1] - robot.joints[ik_chain[i]].xform[1][3]]
        term2[2] = [ee_current[2] - robot.joints[ik_chain[i]].xform[2][3]]
        
        //get local axis in homog coords
        let axis_local = []
        axis_local[0] = [robot.joints[ik_chain[i]].axis[0]]
        axis_local[1] = [robot.joints[ik_chain[i]].axis[1]]
        axis_local[2] = [robot.joints[ik_chain[i]].axis[2]]
        axis_local[3] = [1]

        let z = matrix_multiply(robot.joints[ik_chain[i]].xform, axis_local)
        
        //subtract origin of joint i from axis
        z[0] -= robot.joints[ik_chain[i]].xform[0][3]
        z[1] -= robot.joints[ik_chain[i]].xform[1][3]
        z[2] -= robot.joints[ik_chain[i]].xform[2][3]


        //cross product between axis and diff between ee and joint i
        let v = vector_cross(z,term2)

        if(robot.joints[ik_chain[i]].type == "prismatic"){
            //console.log("prismatic joint")
              
            robot.jacobian[0][i] = z[0]
            robot.jacobian[1][i] = z[1]
            robot.jacobian[2][i] = z[2]
            robot.jacobian[3][i] = 0
            robot.jacobian[4][i] = 0
            robot.jacobian[5][i] = 0
        }

        else if (robot.joints[ik_chain[i]].type == "revolute" || robot.joints[ik_chain[i]].type == "continuous"){
            // //console.log("revolute")
            robot.jacobian[0][i] = v[0]
            robot.jacobian[1][i] = v[1]
            robot.jacobian[2][i] = v[2]
            robot.jacobian[3][i] = z[0]
            robot.jacobian[4][i] = z[1]
            robot.jacobian[5][i] = z[2]
        }
        else{
                // if no joint type is declared assume continuous joint 
                robot.jacobian[0][i] = v[0]
                robot.jacobian[1][i] = v[1]
                robot.jacobian[2][i] = v[2]
                robot.jacobian[3][i] = z[0]
                robot.jacobian[4][i] = z[1]
                robot.jacobian[5][i] = z[2]
        }
        
    }
}
//working on this still
function calculateJacobian_stochastic(ee_current){
    
    //calculate jacobian with respect to joints along the chain from end effector joint to the robot base
    //robot.jacobian = generate2D(6,1) //initialize matrix of required size 
    
    let max = ik_chain.length-1
    let min = 0

    let i =  Math.floor(Math.random() * (max - min) + min)



    // generate values for the jacobian
    //for(i in ik_chain){
        //see slide 64        

        // computing global axis of joint i

        //difference between end effector location and joint i origin in world coords
        let term2 = []
        term2[0] = [ee_current[0] - robot.joints[ik_chain[i]].xform[0][3]]
        term2[1] = [ee_current[1] - robot.joints[ik_chain[i]].xform[1][3]]
        term2[2] = [ee_current[2] - robot.joints[ik_chain[i]].xform[2][3]]
        
        //get local axis in homog coords
        let axis_local = []
        axis_local[0] = [robot.joints[ik_chain[i]].axis[0]]
        axis_local[1] = [robot.joints[ik_chain[i]].axis[1]]
        axis_local[2] = [robot.joints[ik_chain[i]].axis[2]]
        axis_local[3] = [1]

        let z = matrix_multiply(robot.joints[ik_chain[i]].xform, axis_local)
        
        //subtract origin of joint i from axis
        z[0] -= robot.joints[ik_chain[i]].xform[0][3]
        z[1] -= robot.joints[ik_chain[i]].xform[1][3]
        z[2] -= robot.joints[ik_chain[i]].xform[2][3]


        //cross product between axis and diff between ee and joint i
        let v = vector_cross(z,term2)
        
        // console.log("axis_local")
        // console.log(axis_local)
        // console.log("z")
        // console.log(z)
        // console.log("term2")
        // console.log(term2)


        if(robot.joints[ik_chain[i]].type == "prismatic"){
            //console.log("prismatic joint")
              
            robot.jacobian[0][i] = z[0]
            robot.jacobian[1][i] = z[1]
            robot.jacobian[2][i] = z[2]
            robot.jacobian[3][i] = 0
            robot.jacobian[4][i] = 0
            robot.jacobian[5][i] = 0
        }

        else if (robot.joints[ik_chain[i]].type == "revolute" || robot.joints[ik_chain[i]].type == "continuous"){
            // //console.log("revolute")
            robot.jacobian[0][i] = v[0]
            robot.jacobian[1][i] = v[1]
            robot.jacobian[2][i] = v[2]
            robot.jacobian[3][i] = z[0]
            robot.jacobian[4][i] = z[1]
            robot.jacobian[5][i] = z[2]
        }
        else{
                // if no joint type is declared assume continuous joint 
                robot.jacobian[0][i] = v[0]
                robot.jacobian[1][i] = v[1]
                robot.jacobian[2][i] = v[2]
                robot.jacobian[3][i] = z[0]
                robot.jacobian[4][i] = z[1]
                robot.jacobian[5][i] = z[2]
        }
        
    
}