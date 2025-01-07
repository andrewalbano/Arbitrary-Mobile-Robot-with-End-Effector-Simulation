
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints  
    //added extra cases for future implementations if needed 
    switch(kineval.params.dance_sequence_index[kineval.params.dance_pose_index]) {
        case 0:
            kineval.params.setpoint_target = kineval.setpoints[0]
        break

        case 1:
            kineval.params.setpoint_target = kineval.setpoints[1]
        break;

        case 2:
            kineval.params.setpoint_target = kineval.setpoints[2]
        break;

        case 3:
            kineval.params.setpoint_target = kineval.setpoints[3]
        break;

        case 4:
            kineval.params.setpoint_target = kineval.setpoints[4]
        break;

        case 5:
            kineval.params.setpoint_target = kineval.setpoints[5]
        break;

        case 6:
            kineval.params.setpoint_target = kineval.setpoints[6]
        break;

        case 7:
            kineval.params.setpoint_target = kineval.setpoints[7]
        break;
        case 8:
            kineval.params.setpoint_target = kineval.setpoints[8]
        break;
        case 9:
            kineval.params.setpoint_target = kineval.setpoints[9]
        break;

        default:
          
    }
    
    // restart the dance
    if(kineval.params.dance_pose_index > kineval.params.dance_sequence_index.length){
        kineval.params.dance_pose_index = 0 
    }

    /*
    //too slow while screenrecording
        if(kineval.params.dance_pose_index >= 16 ){
            if(magnitudeError < 0.8){
                kineval.params.dance_pose_index = kineval.params.dance_pose_index + 0.5 // we dont know why but 0.5 prevetned it from skipping
                robot.joints[x].servo.p_gain = 0.3
            }
        }   
        else{
            if(magnitudeError < 0.03){
                kineval.params.dance_pose_index = kineval.params.dance_pose_index + 0.5 // we dont know
                robot.joints[x].servo.p_gain =0.03
            }
        }
    */
        // changing speed after num setpoints reached, and iterating to the next waypoint once the error is within a certain amount
        

        //original
        if(kineval.params.dance_pose_index >= 10 ){
            if(magnitudeError < 0.8){
                kineval.params.dance_pose_index = kineval.params.dance_pose_index + 0.5 // we dont know why but 0.5 prevetned it from skipping
                //robot.joints[x].servo.p_gain = 0.5.// not sure if this impacts it becasue of where it is assigned
            }
        }   
        else{
            if(magnitudeError < 0.01){
                kineval.params.dance_pose_index = kineval.params.dance_pose_index + 0.5 // we dont know
                //robot.joints[x].servo.p_gain =0.05
            }
        }
        
            // if(magnitudeError < 0.001){
            //     kineval.params.dance_pose_index = kineval.params.dance_pose_index + 1 // we dont know why but 0.5 prevetned it from skipping
            //     //robot.joints[x].servo.p_gain = 0.5.// not sure if this impacts it becasue of where it is assigned
            // }
       
    
   
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {
    

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    // STENCIL: implement P servo controller over joints


    magnitudeError = 0; // used for determining when to move to teh next waypoint
    for (x in robot.joints) {
        
        robot.joints[x].servo.p_desired = kineval.params.setpoint_target[x] // setting setpoint target
        let error =  robot.joints[x].servo.p_desired - robot.joints[x].angle // computing error 
        robot.joints[x].control = robot.joints[x].servo.p_gain * error //updating the servo control
        
        magnitudeError = (error**2)+ magnitudeError //used to compute euclidean norm running total  //original
        //magnitudeError = error+ magnitudeError


        //debugging
        //console.log("joint:   " + x)
        //console.log("Desired setpoint:  " + robot.joints[x].servo.p_desired + "     current angle:   " + robot.joints[x].angle )
        //console.log("Error :    " + error)
        //console.log("control value :    " + robot.joints[x].control)
        //errorNorm = errorNorm + error
    }
    magnitudeError  = Math.sqrt(magnitudeError) //complete the euclidean norm calculation  //original
   
     
    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

}


