# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

# 1. Model overview: state, actuators and update equations.

The states were set as a vector in length of 6xN + 2x(N-1). The 6xN part stands for the vehicle state (x, y, psi, v, as wellas the error cte and epsi) at 1:N time frames, whereas 2x(N-1) represents the control input at 1:N-1 time frames-- we don't need to include the last step one, as it is not related to our optimization.

       size_t x_start = 0;  // 0 :N-1
       size_t y_start = x_start + N;// N : 2N-1
       size_t psi_start = y_start + N;
       size_t v_start = psi_start + N;
       size_t cte_start = v_start + N;
       size_t epsi_start = cte_start + N;
       size_t delta_start = epsi_start + N;
       size_t a_start = delta_start + N - 1;
 
Then, I set the constrains. For example, two important constrains are the steering angle and acceleration. Typically, the car wheel can shift the steer angle within a range of 30degree. Further due to the actuator limit,the acceleration is also bounded in a range-- strong breaking or throttling is not only uncomfortable, but could also be dangerous. Given the above considerations, I set those two constrains as below.
     
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332 * Lf;
        vars_upperbound[i] =  0.436332 * Lf;
    }
    
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -3.0;
        vars_upperbound[i] =  1.0;
    }

Next, I also need to set the constrains. To ensure the vehicle start at where it should be at t0, I set the upper bound and lower bound to the same value-- equavalently the position and initial state is locked, as below:

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;
    
    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;
 
# 2. N (timestep length) and dt (elapsed duration between timesteps) down selection

I assigned values to N and dt so that to determine how many steps I need to consider in advance to optimize the next steps. First, I don't want to set N to a very large number-- if we were to set N to 100, the simulation would run much slower. Given that in the real driving scenario, the environment is likely to change within 2-3 sec, it is also unrealistic to set dt to very large number. Given that, I set N and dt as below: 

       size_t N = 10;
       double dt = 0.1;

As a result, I am optimizing my MPC in the time scale of 1 sec. Assuming the speed of 40mile/hour, the vehicle will move 18 meter within 1 sec. This is slightly less the the assumed sensor range (25 meter), and would provide a good reference.

# 3. Polynomial is fitted to waypoints.

When polyfitting the trajectory, I first use a small trick to transform the working coordinates to align with the car coordinates by: 
     
     double shift_x = ptsx[i]-px;
     double shift_y = ptsy[i]-py;
     ptsx[i] = (shift_x*cos(0-psi) - shift_y*sin(0-psi));
     ptsy[i] = (shift_x*sin(0-psi) + shift_y*cos(0-psi));


Then I did the polyfit to determinethe coefficients.

     auto coeffs = polyfit(ptsx_transfrom,ptsy_transfrom,3);

This trick benefits me in multiple ways. First, I don't need to worry about the singularity when psi is 90 degree. Second, the cte and epsi is much easier to calculate, as below:

     auto coeffs = polyfit(ptsx_transfrom,ptsy_transfrom,3);
     double cte = polyeval(coeffs,0);
     double epsi = -atan(coeffs[1]);

As a result, I will get the predictions. Of course, only the prediction from the first time frame matters, because all the remainings will be updated again before the second time frame.







