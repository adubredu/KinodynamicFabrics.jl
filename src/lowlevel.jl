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

    τ = zero(q_motors_des) 
    Δq_motors = params[:qmotors] - q_motors_des  
    qdot_motors = qdot[[qleftHipRoll, qleftHipYaw, qleftHipPitch, qleftKnee, qleftToePitch, qleftToeRoll, qrightHipRoll, qrightHipYaw, qrightHipPitch, qrightKnee, qrightToePitch, qrightToeRoll, qleftShoulderRoll, qleftShoulderPitch, qleftShoulderYaw, qleftElbow, qrightShoulderRoll, qrightShoulderPitch, qrightShoulderYaw, qrightElbow]]
    Δq̇_motors = qdot_motors - qdot_motors_des 

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
    β = 1.5*problem.digit.damping 

    if params[:swing_foot] == :right
        τ[LeftHipRoll] = -kp_hiproll_stance * params[:q][qbase_roll] - β[LeftHipRoll]*Δq̇_motors[LeftHipRoll]
        τ[LeftHipYaw] = -kp_hipyaw_stance * Δq_motors[LeftHipYaw] - β[LeftHipYaw]*Δq̇_motors[LeftHipYaw]

        τ[LeftHipPitch] = -kp_hippitch_stance * -params[:q][qbase_pitch] - β[LeftHipPitch]*Δq̇_motors[LeftHipPitch]
        τ[LeftKnee] = -kp_knee_stance * Δq_motors[LeftKnee] - β[LeftKnee]*Δq̇_motors[LeftKnee]

        τ[RightHipRoll] = -kp_hiproll_swing * Δq_motors[RightHipRoll] - β[RightHipRoll]*Δq̇_motors[RightHipRoll]
        τ[RightHipYaw] = -kp_hipyaw_swing * Δq_motors[RightHipYaw]- β[RightHipYaw]*Δq̇_motors[RightHipYaw]
        τ[RightHipPitch] = -kp_hippitch_swing * Δq_motors[RightHipPitch] - β[RightHipPitch]*Δq̇_motors[RightHipPitch]
        τ[RightKnee] = -kp_knee_swing * Δq_motors[RightKnee] - β[RightKnee]*Δq̇_motors[RightKnee]

        τ[LeftToeA] = 0.0
        τ[LeftToeB] = 0.0
        τ[RightToeA] = -kp_toe_swing * Δq_motors[RightToeA]  - β[RightToeA]*0
        τ[RightToeB] = -kp_toe_swing * Δq_motors[RightToeB]  - β[RightToeB]*0

        τ[LeftKnee] += u_knee_comp_
        τ[LeftHipRoll] -= u_hiproll_comp_  
    else
        τ[LeftHipRoll] = -kp_hiproll_swing * Δq_motors[LeftHipRoll]  - β[LeftHipRoll]*Δq̇_motors[LeftHipRoll]
        τ[LeftHipYaw] = -kp_hipyaw_swing * Δq_motors[LeftHipYaw]  - β[LeftHipYaw]*Δq̇_motors[LeftHipYaw]
        τ[LeftHipPitch] = -kp_hippitch_swing * Δq_motors[LeftHipPitch]  - β[LeftHipPitch]*Δq̇_motors[LeftHipPitch]
        τ[LeftKnee] = -kp_knee_swing * Δq_motors[LeftKnee]  - β[LeftKnee]*Δq̇_motors[LeftKnee]

        τ[RightHipRoll] = -kp_hiproll_stance * params[:q][qbase_roll]  - β[RightHipRoll]*Δq̇_motors[RightHipRoll]
        τ[RightHipYaw] = -kp_hipyaw_stance * Δq_motors[RightHipYaw] - β[RightHipYaw]*Δq̇_motors[RightHipYaw]
        τ[RightHipPitch] = -kp_hippitch_stance * params[:q][qbase_pitch]  - β[RightHipPitch]*Δq̇_motors[RightHipPitch]
        τ[RightKnee] = -kp_knee_stance * Δq_motors[RightKnee]  - β[RightKnee]*Δq̇_motors[RightKnee]

        τ[LeftToeA] = -kp_toe_swing * Δq_motors[LeftToeA]  - β[LeftToeA]*0
        τ[LeftToeB] = -kp_toe_swing * Δq_motors[LeftToeB]  - β[LeftToeB]*0
        τ[RightToeA] = 0.0
        τ[RightToeB] = 0.0

        τ[RightKnee] -= u_knee_comp_
        τ[RightHipRoll] += u_hiproll_comp_  
    end 

    τ[LeftShoulderRoll] = -kp_shoulderroll * Δq_motors[LeftShoulderRoll] - β[LeftShoulderRoll] * qdot_motors[LeftShoulderRoll]
    τ[LeftShoulderPitch] = -kp_shoulderpitch * Δq_motors[LeftShoulderPitch]  - β[LeftShoulderPitch] * qdot_motors[LeftShoulderPitch]
    τ[LeftShoulderYaw] = -kp_shoulderyaw * Δq_motors[LeftShoulderYaw] - β[LeftShoulderYaw] * qdot_motors[LeftShoulderYaw]
    τ[LeftElbow] = -kp_elbow * Δq_motors[LeftElbow]
    τ[RightShoulderRoll] = -kp_shoulderroll * Δq_motors[RightShoulderRoll] - β[RightShoulderRoll] * qdot_motors[RightShoulderRoll]
    τ[RightShoulderPitch] = -kp_shoulderpitch * Δq_motors[RightShoulderPitch]  - β[RightShoulderPitch] * qdot_motors[RightShoulderPitch]
    τ[RightShoulderYaw] = -kp_shoulderyaw * Δq_motors[RightShoulderYaw] - β[RightShoulderYaw] * qdot_motors[RightShoulderYaw]
    τ[RightElbow] = -kp_elbow * Δq_motors[RightElbow]  - β[RightElbow] * qdot_motors[RightElbow]
    if save_diagnostics
        push!(problem.task_data[:diagnostics][:qs], Δq_motors[[LeftHipRoll, LeftHipPitch, LeftKnee, RightHipRoll, RightHipPitch, RightKnee]])
        push!(problem.task_data[:diagnostics][:ts], problem.t)
    end  
    return τ
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

    τ = zeros(NUM_MOTORS)  
        
    Δq_motors = q_motors - q_motors_des 
    Δq̇_motors = qdot_motors - qdot_motors_des

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
    if haskey(problem.task_data, :obstacle) kβ = 0.0 else kβ = 0.8 end
    β = kβ*problem.digit.damping 

    τ[LeftHipRoll] = -kp_hiproll_stand * Δq_motors[LeftHipRoll] - β[LeftHipRoll]*Δq̇_motors[LeftHipRoll]
    τ[LeftHipYaw] = -kp_hipyaw_stand * Δq_motors[LeftHipYaw] - β[LeftHipYaw]*Δq̇_motors[LeftHipYaw]
    τ[LeftHipPitch] = -kp_hippitch_stand * Δq_motors[LeftHipPitch] - β[LeftHipPitch]*Δq̇_motors[LeftHipPitch]
    τ[LeftKnee] = -kp_knee_stand * Δq_motors[LeftKnee] - β[LeftKnee]*Δq̇_motors[LeftKnee]
    τ[RightHipRoll] = -kp_hiproll_stand * Δq_motors[RightHipRoll] - β[RightHipRoll]*Δq̇_motors[RightHipRoll]
    τ[RightHipYaw] = -kp_hipyaw_stand * Δq_motors[RightHipYaw] - β[RightHipYaw]*Δq̇_motors[RightHipYaw]
    τ[RightHipPitch] = -kp_hippitch_stand * Δq_motors[RightHipPitch] - β[RightHipPitch]*Δq̇_motors[RightHipPitch]
    τ[RightKnee] = -kp_knee_stand * Δq_motors[RightKnee] - β[RightKnee]*Δq̇_motors[RightKnee]

    τ[LeftShoulderRoll] = -kp_shoulderroll_stand * Δq_motors[LeftShoulderRoll] - β[LeftShoulderRoll]*Δq̇_motors[LeftShoulderRoll]
    τ[LeftShoulderPitch] = -kp_shoulderpitch_stand * Δq_motors[LeftShoulderPitch] - β[LeftShoulderPitch]*Δq̇_motors[LeftShoulderPitch]
    τ[LeftShoulderYaw] = -kp_shoulderyaw_stand * Δq_motors[LeftShoulderYaw] - β[LeftShoulderYaw]*Δq̇_motors[LeftShoulderYaw]
    τ[LeftElbow] = -kp_elbow_stand * Δq_motors[LeftElbow] - β[LeftElbow]*Δq̇_motors[LeftElbow]
    τ[RightShoulderRoll] = -kp_shoulderroll_stand * Δq_motors[RightShoulderRoll] - β[RightShoulderRoll]*Δq̇_motors[RightShoulderRoll]
    τ[RightShoulderPitch] = -kp_shoulderpitch_stand * Δq_motors[RightShoulderPitch] - β[RightShoulderPitch]*Δq̇_motors[RightShoulderPitch]
    τ[RightShoulderYaw] = -kp_shoulderyaw_stand * Δq_motors[RightShoulderYaw] - β[RightShoulderYaw]*Δq̇_motors[RightShoulderYaw]
    τ[RightElbow] = -kp_elbow_stand * Δq_motors[RightElbow] - β[RightElbow]*Δq̇_motors[RightElbow]
 
    τ[LeftToeA] = -kp_toe_stand * Δq_motors[LeftToeA] + kd_toe_stand * v_com_aligned[1] #- β[LeftToeA]*Δq̇_motors[LeftToeA]
    τ[LeftToeB] = -kp_toe_stand * Δq_motors[LeftToeB] - kd_toe_stand * v_com_aligned[1] #- β[LeftToeB]*Δq̇_motors[LeftToeB]
    τ[RightToeA] = -kp_toe_stand * Δq_motors[RightToeA] - kd_toe_stand * v_com_aligned[1] #- β[RightToeA]*Δq̇_motors[RightToeA]
    τ[RightToeB] = -kp_toe_stand * Δq_motors[RightToeB] + kd_toe_stand * v_com_aligned[1] #- β[RightToeB]*Δq̇_motors[RightToeB]

    # knee compensation 
    knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    τ[LeftKnee] += knee_comp
    τ[RightKnee] += knee_comp 
    return τ 
end

function fabric_controller!(digit::Digit; prioritize=true, stand=false)
    q, qdot, qmotors = get_generalized_coordinates(digit) 
    qdes, qdotdes, torqdes = fabric_compute(q, qdot, qmotors, digit.problem;prioritize=prioritize)
    τ = compute_motor_torques(q, qdot, qdes, qdotdes, qmotors, digit.problem, digit.problem.mode)       
    apply_motor_torques!(τ, digit)
    if haskey(digit.problem.task_data, :obstacle) apply_obstacle_force!(digit) end
end

function qp_controller!(digit::Digit)
    q, qdot, qmotors = get_generalized_coordinates(digit) 
    qdes, qdotdes, torqdes = qp_compute(q, qdot, qmotors, digit.problem)
    τ = compute_motor_torques(q, qdot, qdes, qdotdes, qmotors, digit.problem, digit.problem.mode)         
    apply_motor_torques!(τ, digit)
    if pyconvert(Bool, digit.data.time > 1.5)
    apply_obstacle_force!(digit)
    end
end