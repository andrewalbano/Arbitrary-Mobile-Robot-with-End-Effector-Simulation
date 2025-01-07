/*

     KinEval
     Implementation of robot kinematics, control, decision making, and dynamics 
     in HTML5/JavaScript and threejs
     
     @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

*/


kineval.initRobotJoints = function initRobotJoints() {
    // build kinematic hierarchy by looping over each joint in the robot
    //   (object fields can be index through array-style indices, object[field] = property)
    //   and insert threejs scene graph (each joint and link are directly connect to scene root)
    // NOTE: kinematic hierarchy is maintained independently by this code, not threejs

    var x,tempmat;  
    
    for (x in robot.joints) {
        // give the joint its name as an id
        robot.joints[x].name = x;
        
        // initialize joint angle value and control input value
        robot.joints[x].angle = 0;
        robot.joints[x].control = 0;
        robot.joints[x].servo = {};

        //set appropriate servo gains for arm setpoint control
        robot.joints[x].servo.p_gain = 0.1; 
        robot.joints[x].servo.p_desired = 0;
        robot.joints[x].servo.d_gain = 0.01; 
/* STENCIL START */ 
    // STENCIL: complete kinematic hierarchy of robot for convenience.
    //   robot description only specifies parent and child links for joints.
    //   additionally specify parent and child joints for each link
        
        // iterate through each link
        for(y in robot.links){
                  
            // check if children array has been created for the link and if not then create it 
            if (typeof robot.links[y].children == "undefined"){
                robot.links[y].children = []
            }
                
            if (robot.joints[x].child == robot.links[y].name){ // if the joint x child is link y. set link y parent to joint x,
                robot.links[y].parent = robot.joints[x].name // setting the link's parent joint 
                
            }
            if (robot.joints[x].parent == robot.links[y].name){ // if the joint x parent is link y. append joint x to link y children list 
                robot.links[y].children.push(robot.joints[x].name) //appending the link's children joint 
               
            }
            
        }  
     
/* STENCIL END */ 

    }


}





