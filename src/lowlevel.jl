function compute_motor_torques(q, qdot, qdes, qdotdes, q_motors, problem::FabricProblem, mode::WalkingMode; 
                    save_diagnostics=false)         
    params = problem.task_data[:walk]
    min_yaw = deg2rad(-40)
    max_yaw = deg2rad(40)
    st_heading_bos = params[:swing_foot] == :right ? 
    wrap_to_pi(params[:q][qbase_yaw] - params[:q][qleftHipYaw]) : 
    wrap_to_pi(params[:q][qbase_yaw] - params[:q][qrightHipYaw])
    heading_error = wrap_to_pi(params[:target_heading] - st_heading_bos) 
    q_st_yaw_goal = heading_error 

    yaw_error = wrap_to_pi(params[:q][qbase_yaw] - params[:target_heading]) 
    q_sw_yaw_goal = yaw_error 

    q_st_yaw_goal = clamp(q_st_yaw_goal, min_yaw, max_yaw)
    q_sw_yaw_goal = clamp(q_sw_yaw_goal, min_yaw, max_yaw) 

    q_motors_des = zero(params[:qmotors])
    qdot_motors_des = zero(q_motors_des)
    if params[:swing_foot] == :right 
        q_motors_des[LeftKnee] = qdes[qleftKnee]
        q_motors_des[RightHipRoll] = qdes[qrightHipRoll]
        q_motors_des[RightHipPitch] = qdes[qrightHipPitch]
        q_motors_des[RightKnee] = qdes[qrightKnee]

        qdot_motors_des[LeftKnee] = qdotdes[qleftKnee]
        qdot_motors_des[RightHipPitch] = qdotdes[qrightHipPitch]
        qdot_motors_des[RightHipRoll] = qdotdes[qrightHipRoll]
        qdot_motors_des[RightKnee] = qdotdes[qrightKnee]

        q_motors_des[LeftHipYaw] = 0.5*q_st_yaw_goal
        q_motors_des[RightHipYaw] = 0.5*q_sw_yaw_goal
        qdot_motors_des[LeftHipYaw] = 0.0
        qdot_motors_des[LeftHipYaw] = 0.0

        q_motors_des[LeftToeA] = 0.0
        q_motors_des[LeftToeB] = 0.0
        toe_pitch, toe_roll = 0.0, 0.0
        toeA = kin.qm_toe(toe_pitch, toe_roll, "a", "right")
        toeB = kin.qm_toe(toe_pitch, toe_roll, "b", "right")
        q_motors_des[RightToeA] = toeA
        q_motors_des[RightToeB] = toeB
    else 
        q_motors_des[LeftHipRoll] = qdes[qleftHipRoll]
        q_motors_des[LeftHipPitch] = qdes[qleftHipPitch]
        q_motors_des[LeftKnee] = qdes[qleftKnee]
        q_motors_des[RightKnee] = qdes[qrightKnee]

        qdot_motors_des[LeftHipRoll] = qdotdes[qleftHipRoll]
        qdot_motors_des[LeftHipPitch] = qdotdes[qleftHipPitch]
        qdot_motors_des[LeftKnee] = qdotdes[qleftKnee]
        qdot_motors_des[RightKnee] = qdotdes[qrightKnee]

        q_motors_des[LeftHipYaw] = 0.5*q_sw_yaw_goal
        q_motors_des[RightHipYaw] = 0.5*q_st_yaw_goal
        qdot_motors_des[LeftHipYaw] = 0.0
        qdot_motors_des[RightHipYaw] = 0.0

        q_motors_des[RightToeA] = 0.0
        q_motors_des[RightToeB] = 0.0
        toe_pitch, toe_roll = 0.0, 0.0
        toeA = kin.qm_toe(toe_pitch, toe_roll, "a", "left")
        toeB = kin.qm_toe(toe_pitch, toe_roll, "b", "left")
        q_motors_des[LeftToeA] = toeA
        q_motors_des[LeftToeB] = toeB
    end  
    q_motors_des[LeftShoulderRoll] = qdes[qleftShoulderRoll]
    q_motors_des[LeftShoulderPitch] = qdes[qleftShoulderPitch]
    q_motors_des[LeftShoulderYaw] = qdes[qleftShoulderYaw]
    q_motors_des[LeftElbow] = qdes[qleftElbow]
    q_motors_des[RightShoulderRoll] = qdes[qrightShoulderRoll]
    q_motors_des[RightShoulderPitch] = qdes[qrightShoulderPitch]
    q_motors_des[RightShoulderYaw] = qdes[qrightShoulderYaw]
    q_motors_des[RightElbow] = qdes[qrightElbow]

    ?? = zero(q_motors_des) 
    ??q_motors = params[:qmotors] - q_motors_des  
    qdot_motors = qdot[[qleftHipRoll, qleftHipYaw, qleftHipPitch, qleftKnee, qleftToePitch, qleftToeRoll, qrightHipRoll, qrightHipYaw, qrightHipPitch, qrightKnee, qrightToePitch, qrightToeRoll, qleftShoulderRoll, qleftShoulderPitch, qleftShoulderYaw, qleftElbow, qrightShoulderRoll, qrightShoulderPitch, qrightShoulderYaw, qrightElbow]]
    ??q??_motors = qdot_motors - qdot_motors_des 

    kp_hiproll_swing = Walking_Gains[:kp_hiproll_swing] 
    kp_hipyaw_swing = Walking_Gains[:kp_hipyaw_swing]    
    kp_hippitch_swing =Walking_Gains[:kp_hippitch_swing] 
    kp_knee_swing = Walking_Gains[:kp_knee_swing]
    kp_toe_swing = Walking_Gains[:kp_toe_swing]

    kp_hiproll_stance = Walking_Gains[:kp_hiproll_stance]  
    kp_hipyaw_stance = Walking_Gains[:kp_hipyaw_stance]   
    kp_hippitch_stance = Walking_Gains[:kp_hippitch_stance]
    kp_knee_stance = Walking_Gains[:kp_knee_stance]
    kp_toe_stance = Walking_Gains[:kp_toe_stance]

    kp_shoulderroll = Walking_Gains[:kp_shoulderroll]
    kp_shoulderpitch = Walking_Gains[:kp_shoulderpitch]
    kp_shoulderyaw = Walking_Gains[:kp_shoulderyaw]
    kp_elbow = Walking_Gains[:kp_elbow]
    u_knee_comp_ = Walking_Gains[:u_knee_comp_]
    u_hiproll_comp_= Walking_Gains[:u_hiproll_comp_] 
    ?? = 1.5*problem.digit.damping 

    if params[:swing_foot] == :right
        ??[LeftHipRoll] = -kp_hiproll_stance * params[:q][qbase_roll] - ??[LeftHipRoll]*??q??_motors[LeftHipRoll]
        ??[LeftHipYaw] = -kp_hipyaw_stance * ??q_motors[LeftHipYaw] - ??[LeftHipYaw]*??q??_motors[LeftHipYaw]

        ??[LeftHipPitch] = -kp_hippitch_stance * -params[:q][qbase_pitch] - ??[LeftHipPitch]*??q??_motors[LeftHipPitch]
        ??[LeftKnee] = -kp_knee_stance * ??q_motors[LeftKnee] - ??[LeftKnee]*??q??_motors[LeftKnee]

        ??[RightHipRoll] = -kp_hiproll_swing * ??q_motors[RightHipRoll] - ??[RightHipRoll]*??q??_motors[RightHipRoll]
        ??[RightHipYaw] = -kp_hipyaw_swing * ??q_motors[RightHipYaw]- ??[RightHipYaw]*??q??_motors[RightHipYaw]
        ??[RightHipPitch] = -kp_hippitch_swing * ??q_motors[RightHipPitch] - ??[RightHipPitch]*??q??_motors[RightHipPitch]
        ??[RightKnee] = -kp_knee_swing * ??q_motors[RightKnee] - ??[RightKnee]*??q??_motors[RightKnee]

        ??[LeftToeA] = 0.0
        ??[LeftToeB] = 0.0
        ??[RightToeA] = -kp_toe_swing * ??q_motors[RightToeA]  - ??[RightToeA]*0
        ??[RightToeB] = -kp_toe_swing * ??q_motors[RightToeB]  - ??[RightToeB]*0

        ??[LeftKnee] += u_knee_comp_
        ??[LeftHipRoll] -= u_hiproll_comp_  
    else
        ??[LeftHipRoll] = -kp_hiproll_swing * ??q_motors[LeftHipRoll]  - ??[LeftHipRoll]*??q??_motors[LeftHipRoll]
        ??[LeftHipYaw] = -kp_hipyaw_swing * ??q_motors[LeftHipYaw]  - ??[LeftHipYaw]*??q??_motors[LeftHipYaw]
        ??[LeftHipPitch] = -kp_hippitch_swing * ??q_motors[LeftHipPitch]  - ??[LeftHipPitch]*??q??_motors[LeftHipPitch]
        ??[LeftKnee] = -kp_knee_swing * ??q_motors[LeftKnee]  - ??[LeftKnee]*??q??_motors[LeftKnee]

        ??[RightHipRoll] = -kp_hiproll_stance * params[:q][qbase_roll]  - ??[RightHipRoll]*??q??_motors[RightHipRoll]
        ??[RightHipYaw] = -kp_hipyaw_stance * ??q_motors[RightHipYaw] - ??[RightHipYaw]*??q??_motors[RightHipYaw]
        ??[RightHipPitch] = -kp_hippitch_stance * params[:q][qbase_pitch]  - ??[RightHipPitch]*??q??_motors[RightHipPitch]
        ??[RightKnee] = -kp_knee_stance * ??q_motors[RightKnee]  - ??[RightKnee]*??q??_motors[RightKnee]

        ??[LeftToeA] = -kp_toe_swing * ??q_motors[LeftToeA]  - ??[LeftToeA]*0
        ??[LeftToeB] = -kp_toe_swing * ??q_motors[LeftToeB]  - ??[LeftToeB]*0
        ??[RightToeA] = 0.0
        ??[RightToeB] = 0.0

        ??[RightKnee] -= u_knee_comp_
        ??[RightHipRoll] += u_hiproll_comp_  
    end 

    ??[LeftShoulderRoll] = -kp_shoulderroll * ??q_motors[LeftShoulderRoll] - ??[LeftShoulderRoll] * qdot_motors[LeftShoulderRoll]
    ??[LeftShoulderPitch] = -kp_shoulderpitch * ??q_motors[LeftShoulderPitch]  - ??[LeftShoulderPitch] * qdot_motors[LeftShoulderPitch]
    ??[LeftShoulderYaw] = -kp_shoulderyaw * ??q_motors[LeftShoulderYaw] - ??[LeftShoulderYaw] * qdot_motors[LeftShoulderYaw]
    ??[LeftElbow] = -kp_elbow * ??q_motors[LeftElbow]
    ??[RightShoulderRoll] = -kp_shoulderroll * ??q_motors[RightShoulderRoll] - ??[RightShoulderRoll] * qdot_motors[RightShoulderRoll]
    ??[RightShoulderPitch] = -kp_shoulderpitch * ??q_motors[RightShoulderPitch]  - ??[RightShoulderPitch] * qdot_motors[RightShoulderPitch]
    ??[RightShoulderYaw] = -kp_shoulderyaw * ??q_motors[RightShoulderYaw] - ??[RightShoulderYaw] * qdot_motors[RightShoulderYaw]
    ??[RightElbow] = -kp_elbow * ??q_motors[RightElbow]  - ??[RightElbow] * qdot_motors[RightElbow]
    if save_diagnostics
        push!(problem.task_data[:diagnostics][:qs], ??q_motors[[LeftHipRoll, LeftHipPitch, LeftKnee, RightHipRoll, RightHipPitch, RightKnee]])
        push!(problem.task_data[:diagnostics][:ts], problem.t)
    end  
    return ??
end

function compute_motor_torques(q, qdot, qdes, qdotdes, q_motors, problem::FabricProblem, mode::StandingMode)         
    heading = q[qbase_yaw] 
    Rz = RotZ(heading)  

    # Aligned toes
    p_left_toe_world = kin.p_toe_pitch_joint_left(q)
    p_right_toe_world = kin.p_toe_pitch_joint_right(q)
    p_left_toe_aligned = Rz' * p_left_toe_world 
    p_right_toe_aligned = Rz' * p_right_toe_world

    # Aligned com 
    p_com_world = kin.p_COM(q)
    v_com_world = kin.v_COM(q, qdot)
    p_com_aligned =  Rz' * p_com_world
    v_com_aligned =  Rz' * v_com_world   

    q_motors_des = copy(q_motors)
    qdot_motors_des = zeros(NUM_MOTORS) 

    q_motors_des[LeftHipRoll] = qdes[qleftHipRoll]
    q_motors_des[LeftHipPitch] = qdes[qleftHipPitch]
    q_motors_des[LeftKnee] = qdes[qleftKnee]
    q_motors_des[RightHipRoll] = qdes[qrightHipRoll]
    q_motors_des[RightHipPitch] = qdes[qrightHipPitch]
    q_motors_des[RightKnee] = qdes[qrightKnee]
    q_motors_des[LeftHipYaw] = 0
    q_motors_des[RightHipYaw] = 0
    q_motors_des[LeftShoulderRoll] = qdes[qleftShoulderRoll]
    q_motors_des[LeftShoulderPitch] = qdes[qleftShoulderPitch]
    q_motors_des[LeftShoulderYaw] = qdes[qleftShoulderYaw]
    q_motors_des[LeftElbow] = qdes[qleftElbow]
    q_motors_des[RightShoulderRoll] = qdes[qrightShoulderRoll]
    q_motors_des[RightShoulderPitch] = qdes[qrightShoulderPitch]
    q_motors_des[RightShoulderYaw] = qdes[qrightShoulderYaw]
    q_motors_des[RightElbow] = qdes[qrightElbow]          
    qdot_motors_des[LeftHipRoll] = qdotdes[qleftHipRoll]
    qdot_motors_des[LeftHipPitch] = qdotdes[qleftHipPitch]
    qdot_motors_des[LeftKnee] = qdotdes[qleftKnee]
    qdot_motors_des[RightHipRoll] = qdotdes[qrightHipRoll]
    qdot_motors_des[RightHipPitch] = qdotdes[qrightHipPitch]
    qdot_motors_des[RightKnee] = qdotdes[qrightKnee] 
    
    if haskey(problem.task_data, :hoop)
        if problem.task_data[:hoop][:fling] 
            qdot_motors_des[RightElbow]  = problem.task_data[:hoop][:throw_torque]
            qdot_motors_des[RightShoulderPitch]  = problem.task_data[:hoop][:throw_torque]
        end
    end
    if haskey(problem.task_data, :cornhole) 
        if problem.task_data[:cornhole][:fling] 
            qdot_motors_des[RightShoulderPitch]  = problem.task_data[:cornhole][:throw_torque]
        end
    end 

    com_midpoint_error = p_com_aligned - 0.5 * (p_left_toe_aligned + p_right_toe_aligned)
    toe_pitch_error = com_midpoint_error[1] 
    Kerr = 1.3 
    toe_pitch = qdes[qleftToePitch] - Kerr*toe_pitch_error
    toe_roll = qdes[qleftToeRoll]
    rtoeA = kin.qm_toe(-toe_pitch, -toe_roll, "a", "right")
    rtoeB = kin.qm_toe(-toe_pitch, -toe_roll, "b", "right")
    ltoeA = kin.qm_toe(toe_pitch, toe_roll, "a", "left")
    ltoeB = kin.qm_toe(toe_pitch, toe_roll, "b", "left")
    q_motors_des[RightToeA] = rtoeA
    q_motors_des[RightToeB] = rtoeB
    q_motors_des[LeftToeA] = ltoeA
    q_motors_des[LeftToeB] = ltoeB
    qdot_motors = qdot[[qleftHipRoll, qleftHipYaw, qleftHipPitch, qleftKnee, qleftToePitch, qleftToeRoll, qrightHipRoll, qrightHipYaw, qrightHipPitch, qrightKnee, qrightToePitch, qrightToeRoll, qleftShoulderRoll, qleftShoulderPitch, qleftShoulderYaw, qleftElbow, qrightShoulderRoll, qrightShoulderPitch, qrightShoulderYaw, qrightElbow]]

    ?? = zeros(NUM_MOTORS)  
        
    ??q_motors = q_motors - q_motors_des 
    ??q??_motors = qdot_motors - qdot_motors_des

    kp_hiproll_stand = Standing_Gains[:kp_hiproll_stand]   
    kp_hipyaw_stand = Standing_Gains[:kp_hipyaw_stand]
    kp_hippitch_stand = Standing_Gains[:kp_hippitch_stand]
    kp_knee_stand = Standing_Gains[:kp_knee_stand]
    kp_toe_stand = Standing_Gains[:kp_toe_stand]  
    kd_toe_stand = Standing_Gains[:kd_toe_stand]
    kp_knee_comp_stand = Standing_Gains[:kp_knee_comp_stand]
    kd_knee_comp_stand = Standing_Gains[:kd_knee_comp_stand]

    kp_shoulderroll_stand = Standing_Gains[:kp_shoulderroll_stand]
    kp_shoulderpitch_stand = Standing_Gains[:kp_shoulderpitch_stand]
    kp_shoulderyaw_stand = Standing_Gains[:kp_shoulderyaw_stand]
    kp_elbow_stand = Standing_Gains[:kp_elbow_stand]
    if haskey(problem.task_data, :obstacle) k?? = 0.0 else k?? = 0.8 end
    ?? = k??*problem.digit.damping 

    ??[LeftHipRoll] = -kp_hiproll_stand * ??q_motors[LeftHipRoll] - ??[LeftHipRoll]*??q??_motors[LeftHipRoll]
    ??[LeftHipYaw] = -kp_hipyaw_stand * ??q_motors[LeftHipYaw] - ??[LeftHipYaw]*??q??_motors[LeftHipYaw]
    ??[LeftHipPitch] = -kp_hippitch_stand * ??q_motors[LeftHipPitch] - ??[LeftHipPitch]*??q??_motors[LeftHipPitch]
    ??[LeftKnee] = -kp_knee_stand * ??q_motors[LeftKnee] - ??[LeftKnee]*??q??_motors[LeftKnee]
    ??[RightHipRoll] = -kp_hiproll_stand * ??q_motors[RightHipRoll] - ??[RightHipRoll]*??q??_motors[RightHipRoll]
    ??[RightHipYaw] = -kp_hipyaw_stand * ??q_motors[RightHipYaw] - ??[RightHipYaw]*??q??_motors[RightHipYaw]
    ??[RightHipPitch] = -kp_hippitch_stand * ??q_motors[RightHipPitch] - ??[RightHipPitch]*??q??_motors[RightHipPitch]
    ??[RightKnee] = -kp_knee_stand * ??q_motors[RightKnee] - ??[RightKnee]*??q??_motors[RightKnee]

    ??[LeftShoulderRoll] = -kp_shoulderroll_stand * ??q_motors[LeftShoulderRoll] - ??[LeftShoulderRoll]*??q??_motors[LeftShoulderRoll]
    ??[LeftShoulderPitch] = -kp_shoulderpitch_stand * ??q_motors[LeftShoulderPitch] - ??[LeftShoulderPitch]*??q??_motors[LeftShoulderPitch]
    ??[LeftShoulderYaw] = -kp_shoulderyaw_stand * ??q_motors[LeftShoulderYaw] - ??[LeftShoulderYaw]*??q??_motors[LeftShoulderYaw]
    ??[LeftElbow] = -kp_elbow_stand * ??q_motors[LeftElbow] - ??[LeftElbow]*??q??_motors[LeftElbow]
    ??[RightShoulderRoll] = -kp_shoulderroll_stand * ??q_motors[RightShoulderRoll] - ??[RightShoulderRoll]*??q??_motors[RightShoulderRoll]
    ??[RightShoulderPitch] = -kp_shoulderpitch_stand * ??q_motors[RightShoulderPitch] - ??[RightShoulderPitch]*??q??_motors[RightShoulderPitch]
    ??[RightShoulderYaw] = -kp_shoulderyaw_stand * ??q_motors[RightShoulderYaw] - ??[RightShoulderYaw]*??q??_motors[RightShoulderYaw]
    ??[RightElbow] = -kp_elbow_stand * ??q_motors[RightElbow] - ??[RightElbow]*??q??_motors[RightElbow]
 
    ??[LeftToeA] = -kp_toe_stand * ??q_motors[LeftToeA] + kd_toe_stand * v_com_aligned[1] #- ??[LeftToeA]*??q??_motors[LeftToeA]
    ??[LeftToeB] = -kp_toe_stand * ??q_motors[LeftToeB] - kd_toe_stand * v_com_aligned[1] #- ??[LeftToeB]*??q??_motors[LeftToeB]
    ??[RightToeA] = -kp_toe_stand * ??q_motors[RightToeA] - kd_toe_stand * v_com_aligned[1] #- ??[RightToeA]*??q??_motors[RightToeA]
    ??[RightToeB] = -kp_toe_stand * ??q_motors[RightToeB] + kd_toe_stand * v_com_aligned[1] #- ??[RightToeB]*??q??_motors[RightToeB]

    # knee compensation 
    knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    ??[LeftKnee] += knee_comp
    ??[RightKnee] += knee_comp 
    return ?? 
end

function fabric_controller!(digit::Digit; prioritize=true, stand=false)
    q, qdot, qmotors = get_generalized_coordinates(digit) 
    qdes, qdotdes, torqdes = fabric_compute(q, qdot, qmotors, digit.problem;prioritize=prioritize)
    ?? = compute_motor_torques(q, qdot, qdes, qdotdes, qmotors, digit.problem, digit.problem.mode)       
    apply_motor_torques!(??, digit)
    if haskey(digit.problem.task_data, :obstacle) apply_obstacle_force!(digit) end
end

function qp_controller!(digit::Digit)
    q, qdot, qmotors = get_generalized_coordinates(digit) 
    qdes, qdotdes, torqdes = qp_compute(q, qdot, qmotors, digit.problem)
    ?? = compute_motor_torques(q, qdot, qdes, qdotdes, qmotors, digit.problem, digit.problem.mode)         
    apply_motor_torques!(??, digit)
    if pyconvert(Bool, digit.data.time > 1.5)
    apply_obstacle_force!(digit)
    end
end