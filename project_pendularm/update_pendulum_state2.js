function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
    pendulum.angle[0] = (pendulum.angle[0] + pendulum.angle_dot[0]*dt)//% (2 * Math.PI)
    pendulum.angle[1] = (pendulum.angle[1] + pendulum.angle_dot[1]*dt)//% (2 * Math.PI)
    pendulum.angle_dot[0] = pendulum.angle_dot[0] + (pendulum.angle_dot_dot[0] * dt)
    pendulum.angle_dot[1] = pendulum.angle_dot[1] + (pendulum.angle_dot_dot[1] * dt)    
   
    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment

    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator

    
    // STENCIL: Runge-Kutta 4 integrator
    let kx1 = pendulum.angle
    let kv1 = pendulum.angle_dot
    let ka1 = pendulum_acceleration(pendulum,gravity)
    
    //k2
    
    let kx2 = [kx1[0] + ((1/2)*kv1[0]*dt) , kx1[1] + ((1/2)*kv1[1]*dt)]
    let kv2 = [kv1[0] + ((1/2)*ka1[0]*dt) , kv1[1] + ((1/2)*ka1[1]*dt)]
    
    //updating pendulum state   
    pendulum.angle = kx2
    pendulum.angle_dot = kv2
    let ka2 = pendulum_acceleration(pendulum,gravity)
    
    // k3 
    //let kx3 = kx1 + ((1/2)*kv2*dt)
    //let kv3 = kv1 + ((1/2)*ka2*dt)
    
    let kx3 = [kx1[0] + ((1/2)*kv2[0]*dt) , kx1[1] + ((1/2)*kv2[1]*dt)]
    let kv3 = [kv1[0] + ((1/2)*ka2[0]*dt) , kv1[1] + ((1/2)*ka2[1]*dt)]
    
    //updating pendulum state
    pendulum.angle = kx3
    pendulum.angle_dot = kv3
    let ka3 = pendulum_acceleration(pendulum,gravity)
    
    // k4
    //    let kx4 = kx1 +(1*kv3*dt)
    //let kv4 = kv1 +(1*ka3*dt)
    let kx4 = [kx1[0] + ((1/2)*kv3[0]*dt) , kx1[1] + ((1/2)*kv3[1]*dt)]
    let kv4 = [kv1[0] + ((1/2)*ka3[0]*dt) , kv1[1] + ((1/2)*ka3[1]*dt)]

    //updating pendulum state
    pendulum.angle = kx4
    pendulum.angle_dot = kv4
    let ka4 = pendulum_acceleration(pendulum,gravity)

    //computing true theta and theta dot       
    pendulum.angle[0] = kx1[0] +dt*((kv1[0]/6) + (kv2[0]/3) + (kv3[0]/3) +(kv4[0]/6))
    pendulum.angle[1] = kx1[1] +dt*((kv1[1]/6) + (kv2[1]/3) + (kv3[1]/3) +(kv4[1]/6))

    pendulum.angle_dot[0] = kv1[0] +dt*((ka1[0]/6) + (ka2[0]/3) + (ka3[0]/3) +(ka4[0]/6))
    pendulum.angle_dot[1] = kv1[1] +dt*((ka1[1]/6) + (ka2[1]/3) + (ka3[1]/3) +(ka4[1]/6))
   

    } 
    else {
        pendulum.angle_previous[0] = pendulum.angle[0];
        pendulum.angle[0] = (pendulum.angle[0]+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[0] = (pendulum.angle[0]-pendulum.angle_previous[0])/dt;
        pendulum.angle_previous[1] = pendulum.angle[1];
        pendulum.angle[1] = (pendulum.angle[1]-Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[1] = (pendulum.angle[1]-pendulum.angle_previous[1])/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    
    let term_1 = pendulum.mass[1]*pendulum.length[0]*pendulum.length[1]*(pendulum.angle_dot[0]**2)*Math.sin(pendulum.angle[0]-pendulum.angle[1])
    let term_2 = pendulum.mass[1] * gravity * pendulum.length[1] * Math.sin(pendulum.angle[1])
    let term_3 = pendulum.control[1]
    let term_4 = pendulum.mass[1] * pendulum.length[1] * (pendulum.angle[1]**2) * Math.sin(pendulum.angle[0]-pendulum.angle[1])
    let term_5 = (pendulum.mass[0]+pendulum.mass[1])* gravity * pendulum.length[0] * Math.sin(pendulum.angle[0])

    let numerator_1 = (-(term_1 - term_2 + term_3) * Math.cos(pendulum.angle[0] - pendulum.angle[1])) -term_4 - term_5 + pendulum.control[0]
    let denominator_1 = (pendulum.mass[0] + pendulum.mass[1] *(pendulum.length[0]**2)) - (pendulum.mass[1] * pendulum.length[0] * pendulum.length[1] * (Math.cos(pendulum.angle[0]-pendulum.angle[1]))**2)
    
    let a_1 = numerator_1/denominator_1


    let term_6 = pendulum.mass[1] * pendulum.length[0] * pendulum.length[1] * a_1 * Math.cos(pendulum.angle[0]-pendulum.angle[1])
    let term_7 = pendulum.mass[1] * pendulum.length[0] * pendulum.length[1] * (pendulum.angle_dot[0]**2) * Math.sin(pendulum.angle[0]-pendulum.angle[1])
    let term_8 = pendulum.mass[1] * gravity * pendulum.length[1] * Math.sin(pendulum.angle[1])

    let numerator_2 = -term_6 + term_7 - term_8 + pendulum.control[1]
    let denominator_2 = pendulum.mass[1]*pendulum.length[1]

    let a_2 = numerator_2 / denominator_2
    
    return [a_1, a_2]
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time

    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:[0,0], kd:[0,0], ki:[0,0]};  // no control
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error

    return [pendulum, accumulated_error];
}