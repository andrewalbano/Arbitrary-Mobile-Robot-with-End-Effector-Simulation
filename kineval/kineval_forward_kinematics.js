
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 
    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: call kineval.buildFKTransforms();
    kineval.buildFKTransforms()
    
}

    // STENCIL: implement buildFKTransforms, which kicks off
    //   a recursive traversal over links and 
    //   joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // To use the keyboard interface, assign the global variables 
    //   "robot_heading" and "robot_lateral", 
    //   which represent the z-axis (heading) and x-axis (lateral) 
    //   of the robot's base in its own reference frame, 
    //   transformed into the world coordinates.
    // The axes should be represented in unit vector form 
    //   as 4x1 homogenous matrices

    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //robot.joints['JointX'].origin.xyz and robot.joints['JointX'].origin.rpy,

    //helper function 
    function peek(stack){
        // returns the value at the top of the stack without deleting it
        return stack[stack.length-1]
    }


    function traverseFKBase(){
        
        let mstack = peek(stack) // getting matrix at top of stack
                
        robot.links[robot.base].xform = mstack // transform of link wrt. world

        traverseFKJoint(robot.links[robot.base].children) // traversing to child joints 

        stack.pop() // pop after all children are explored

    }
    
    
    function traverseFKJoint(joint){
        
        for(i in joint){ 
            /*
            //syntax notes:
            compute transform wrt to parent link 
            robot.joints[joint[i]].origin.xyz
            robot.joints[joint[i]].origin.rpy
            */

            let mstack = peek(stack) // get top of stack 

        
            //compute local transform
            let DL = generate_translation_matrix(robot.joints[joint[i]].origin.xyz[0], robot.joints[joint[i]].origin.xyz[1], robot.joints[joint[i]].origin.xyz[2])
            let Rx = generate_rotation_matrix_X(robot.joints[joint[i]].origin.rpy[0])
            let Ry = generate_rotation_matrix_Y(robot.joints[joint[i]].origin.rpy[1])
            let Rz = generate_rotation_matrix_Z(robot.joints[joint[i]].origin.rpy[2])
                
            let RL = matrix_multiply(Rz,Ry)
            RL = matrix_multiply(RL,Rx)
            let local_transform = matrix_multiply(DL,RL)
                
            mstack = matrix_multiply(mstack,local_transform) //update mstack to get new transform            

            // accounting for motors at the joints
            if( robot.joints[joint[i]].type == "prismatic"){
                     
                //getting unit joint axis
                let unitJointAxis = vector_normalize(robot.joints[joint[i]].axis)

                // generating translation matrix in direction of unit joint axis
                let motorTransform = generate_translation_matrix(unitJointAxis[0], unitJointAxis[1], unitJointAxis[2]) 
                
                //scaling the translation using the angle... (angle in this sense refers to the amount of translation, think of product of exponential definition for theta)
                motorTransform[0][3] *= robot.joints[joint[i]].angle
                motorTransform[1][3] *= robot.joints[joint[i]].angle
                motorTransform[2][3] *= robot.joints[joint[i]].angle

                //getting the new value for the stack 
                mstack = matrix_multiply(mstack,motorTransform)
                
            }
            else if( robot.joints[joint[i]].type == "continuous"){
                
                let q = kineval.quaternionFromAxisAngle(robot.joints[joint[i]].axis,robot.joints[joint[i]].angle)
                q = kineval.quaternionNormalize(q)
                let motorTransform = kineval.quaternionToRotationMatrix(q)
                mstack = matrix_multiply(mstack,motorTransform)
            }
            else if( robot.joints[joint[i]].type == "revolute"){
            
                let q = kineval.quaternionFromAxisAngle(robot.joints[joint[i]].axis, robot.joints[joint[i]].angle)
                q = kineval.quaternionNormalize(q)
                let motorTransform = kineval.quaternionToRotationMatrix(q)
                mstack = matrix_multiply(mstack,motorTransform)
            }
            else if(robot.joints[joint[i]].type == "fixed"){
                mstack = mstack
            }
            else{ // assume revolute if joint is not specified
                
                let q = kineval.quaternionFromAxisAngle(robot.joints[joint[i]].axis, robot.joints[joint[i]].angle)
                q = kineval.quaternionNormalize(q)
                let motorTransform = kineval.quaternionToRotationMatrix(q)
                mstack = matrix_multiply(mstack,motorTransform)
            }
            
            stack.push(mstack) //add updated transform to the stack         
                    
            robot.joints[joint[i]].xform = mstack // transform of joint wrt. world
 
            //recurse to child link 
            traverseFKLink(robot.joints[joint[i]].child) // recurse to joint child
       }
        
   }
    

    function traverseFKLink(link){
        let mstack = peek(stack) // get matrix at top of stack
            
        robot.links[link].xform = mstack // transform of link wrt. world
       
        //if it is a leaf node, pop it 
        if(robot.links[link].children.length == 0){
            stack.pop()
        }
        else{ // continue recursing and pop when finished recursing
            traverseFKJoint(robot.links[link].children)
            stack.pop()
        }
        
    }

    kineval.buildFKTransforms = function buildFKTransforms() {  

        //world frame, root of kinematic tree
        stack = [generate_identity()] //initialized top of stack with identity      

        //Traversed to global frame
        /* notes:
            robot.origin.xyz //robot’s position wrt. world
            robot.origin.rpy //robot’s orientation wrt. world
        */

        let mstack = peek(stack) // getting the matrix at the top of the stack


        //computing transform of global wrt to world 
        let DW = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2])
        let Rx = generate_rotation_matrix_X(robot.origin.rpy[0])
        let Ry = generate_rotation_matrix_Y(robot.origin.rpy[1])
        let Rz = generate_rotation_matrix_Z(robot.origin.rpy[2])
        
            
        let RW = matrix_multiply(Rz,Ry)
        RW = matrix_multiply(RW,Rx)

        let local_transform = matrix_multiply(DW,RW)

        mstack = matrix_multiply(mstack,local_transform) //updating transform


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
           
            robot_lateral = matrix_multiply(mstack,yaxis)//used for left right... // my xaxis is their y axis 
            robot_heading= matrix_multiply(mstack,xaxis) //used for forward back // my zaxis is theie x axis
            
        }
        else{
            //robot heading and lateral
            //in our coordinate system.. // these are used for robot lateral and heading
            let xaxis = [[-1],[0],[0],[1]] // negative sign accounts for the code given in user input
            let zaxis = [[0],[0],[1],[1]]
            robot_lateral= matrix_multiply(mstack,xaxis) //used for left right
            robot_heading = matrix_multiply(mstack,zaxis)//used for forward back
        }
        
        stack.push(mstack) // storing the global transform at the top of the stack
        robot.origin.xform = mstack // this is the base transform
     

        // recurse to child link, which is robot.base
        traverseFKBase()
        stack = [] //emptying the stack... alternatively pop twice

    }
    