function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30
    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
    pendulum.angle = (pendulum.angle + pendulum.angle_dot*dt)//% (2 * Math.PI)
    pendulum.angle_dot = pendulum.angle_dot + (pendulum.angle_dot_dot * dt)
    }

   else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration   
    let temp = pendulum.angle
    pendulum.angle = (2 * pendulum.angle) - (pendulum.angle_previous) + (pendulum.angle_dot_dot*dt**2)//% (2 * Math.PI)
    pendulum.angle_previous = temp

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
    pendulum.angle_previous = pendulum.angle

    pendulum.angle = pendulum.angle + (pendulum.angle_dot * dt) + ((pendulum.angle_dot_dot*dt**2)/2)

    pendulum.angle_dot_dot_future = pendulum_acceleration(pendulum,gravity)

    pendulum.angle_dot = pendulum.angle_dot + ((pendulum.angle_dot_dot + pendulum.angle_dot_dot_future)/2)*dt

    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
    let kx1 = pendulum.angle
    let kv1 = pendulum.angle_dot
    let ka1 = pendulum_acceleration(pendulum,gravity)
    //k2
    
    let kx2 = kx1 + ((1/2)*kv1*dt)
    let kv2 = kv1 + ((1/2)*ka1*dt)
    
    //updating pendulum state
    pendulum.angle = kx2
    pendulum.angle_dot = kv2 
    let ka2 = pendulum_acceleration(pendulum,gravity)
    
    // k3 
    let kx3 = kx1 + ((1/2)*kv2*dt)
    let kv3 = kv1 + ((1/2)*ka2*dt)
    
    //updating pendulum state
    pendulum.angle = kx3
    pendulum.angle_dot = kv3
    let ka3 = pendulum_acceleration(pendulum,gravity)
    
    // k4
    let kx4 = kx1 +(1*kv3*dt)
    let kv4 = kv1 +(1*ka3*dt)

    //updating pendulum state
    pendulum.angle = kx4
    pendulum.angle_dot = kv4
    let ka4 = pendulum_acceleration(pendulum,gravity)

    //computing true theta and theta dot
    pendulum.angle = kx1 +dt*((kv1/6) + (kv2/3) + (kv3/3) +(kv4/6))
    pendulum.angle_dot = kv1 +dt*((ka1/6) + (ka2/3) + (ka3/3) +(ka4/6))
    
    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    return -(gravity/pendulum.length)*Math.sin(pendulum.angle)+ (pendulum.control/(pendulum.mass*pendulum.length**2));
}   

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time

    pendulum.angle_dot_dot = pendulum_acceleration(pendulum, gravity)
       
    t = t + dt
    pendulum.angle_previous = pendulum.angle
    pendulum.angle = pendulum.angle + (dt*pendulum.angle_dot_dot) + ((pendulum.angle_dot_dot * dt**2)/2)

    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid prameters
    pendulum.servo = {kp:75, kd: 70, ki: 2};  // no control
    /*good results:*/
    /*
    {kp:100, kd: 100, ki: 0.5} = 21 in 840 seconds, when i stopped it, by far the best so far, maybe try slightly higher i for integrator to be better at steady state error
    
    {kp:100, kd: 100, ki: 1} TIME TO REACH 5 SETPOINTS = 170
    
    {kp:100, kd: 100, ki: 0.7} time to reach 5 setpoints = 189

    {kp:100, kd: 100, ki: 0.5} 200 secs to get to 5

    {kp:100, kd: 100, ki: 1.5}  127 secs to get to 5 setpoints

    {kp:100, kd: 100, ki: 2} 130 to get to 5

    {kp:100, kd: 100, ki: 3} to get to 5 115

    {kp:100, kd: 100, ki: 10} 110

    {kp:100, kd: 100, ki: 15} get some overshoot 120

    {kp:100, kd: 110, ki: 15} 105 stil a bit of overshoot and 
    
    {kp:150, kd: 135, ki: 20}; 115 to 5 needs more dampening
    {kp:120, kd: 105, ki: 2}; 110
    {kp:120, kd: 100, ki: 3} 115
    {kp:110, kd: 85, ki: 2.5}; 110
    {kp:105, kd: 70, ki: 2.8}; 108
    {kp:105, kd: 65, ki: 2.8}; 106
    {kp:80, kd: 40, ki: 2.8}; 100
    */
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error
    
    // check the first time if error is defined
    var errorIsDeclared = true; 
    try{ error; }
    catch(e) {
        if(e.name == "ReferenceError") {
            errorIsDeclared = false;
        }
    }

    if(errorIsDeclared){
        error_previous = error
        error = pendulum.desired - pendulum.angle
        
    }
    else{
        error_previous = 0
        error = 0
    }

    

    
    accumulated_error = error + accumulated_error
    pendulum.control = (pendulum.servo.kp * error) + (pendulum.servo.ki * accumulated_error) +(pendulum.servo.kd * ((error - error_previous)/dt))
    
    return [pendulum, accumulated_error];
}