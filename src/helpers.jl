# helpers 
function reach_map(x, params::Dict)
    res = x - params[:goal] 
    return res 
end

function reach_force(x, params::Dict)
    K = params[:K] 
    f = -K*x 
    return f 
end

function get_floating_pose(θ, params::Dict)
    p = θ[[di.qbase_pos_x, di.qbase_pos_y, di.qbase_yaw]]-params[:init_position]
    return p
end

function filter_coordinates(vs, prob, val=:vel)
    param = prob.task_data[:filter]
    a = param[:filter_parameter]
    if val == :vel
        if param[:first_iter_vel]
            qdot_filtered = vs
            param[:qdot_filtered] = vs
            param[:first_iter_vel] = false
        else
            qdot_filtered = (1-a)*param[:qdot_filtered] + a*vs
            param[:qdot_filtered] = qdot_filtered
        end
        return qdot_filtered
    else
        if param[:first_iter_pos]
            q_filtered = vs
            param[:q_filtered] = vs
            param[:first_iter_pos] = false
        else
            q_filtered = (1-a)*param[:q_filtered] + a*vs
            param[:q_filtered] = q_filtered
        end
        return q_filtered
    end

end

function default_ik(q, com_goal, digit; 
    max_iter=200, tolerance=1e-3, maxΔ=0.1)
    θ = copy(q)
    leg_indices = [qleftHipRoll, qleftHipPitch, qleftKnee, qrightHipRoll, qrightHipPitch, qrightKnee]  
    θ[qbase_pitch] = 0.0
    θ[qbase_roll] = 0.0
    θ[qleftHipYaw] = 0.0 
    θ[qrightHipYaw] = 0.0   
    iter = 1
    for _ = 1:max_iter
        θ[qleftShinToTarsus] = -θ[qleftKnee]
        θ[qrightShinToTarsus] = -θ[qrightKnee]
        com_pos = kin.p_com_wrt_feet(θ)
        com_error = com_pos - com_goal 
        error = norm(com_error, Inf) 
        if error < tolerance 
            # printstyled("converged at iter $iter\n";color=:blue)
            break
        end
        Jall = kin.Jp_com_wrt_feet(θ) 
        J = Jall[1:end, leg_indices]
        J[1:end, 3] -= Jall[1:end, qleftShinToTarsus]
        J[1:end, 6] -= Jall[1:end, qrightShinToTarsus]
        θΔ = J \ com_error
        maxofD = max(θΔ...)
        if maxofD > maxΔ θΔ = maxΔ * (θΔ /maxofD) end 
        θ[leg_indices] -= θΔ   
        θ[leg_indices] = clamp.(θ[leg_indices], digit.θ_min[leg_indices], digit.θ_max[leg_indices])
        iter+=1
    end
    if iter > max_iter printstyled("Did not converge"; bold=true, color=:red) end 
    return θ
end

function compute_default_torques(θ, θ̇ , q_motors, problem; com_goal=[0.0,0.0,0.95])  
    step_width = 0.3 
    heading = θ[qbase_yaw]   
    Rz = RotZ(heading)  
    
    p_left_toe_world = kin.p_toe_pitch_joint_left(θ)
    p_right_toe_world = kin.p_toe_pitch_joint_right(θ)
    p_left_toe_aligned = Rz' * p_left_toe_world 
    p_right_toe_aligned = Rz' * p_right_toe_world

    p_com_world = kin.p_COM(θ)
    v_com_world = kin.v_COM(θ, θ̇ )
    p_com_aligned =  Rz' * p_com_world
    v_com_aligned =  Rz' * v_com_world  
 
    com_wrt_left_aligned_des = [0.0, -0.5*step_width, com_goal[3]]
    com_wrt_right_aligned_des = [0.0, 0.5*step_width, com_goal[3]]
    goal =  [(Rz * com_wrt_left_aligned_des)..., (Rz * com_wrt_right_aligned_des)...] 

    θd = default_ik(θ, goal, problem.digit)   
    q_motors_des = copy(q_motors) 

    q_motors_des[LeftHipRoll] = θd[qleftHipRoll]
    q_motors_des[LeftHipPitch] = θd[qleftHipPitch]
    q_motors_des[LeftKnee] = θd[qleftKnee]
    q_motors_des[RightHipRoll] = θd[qrightHipRoll]
    q_motors_des[RightHipPitch] = θd[qrightHipPitch]
    q_motors_des[RightKnee] = θd[qrightKnee]

    q_motors_des[LeftHipYaw] = 0.0
    q_motors_des[RightHipYaw] = 0.0

    q_motors_des[LeftShoulderRoll] = -0.15
    q_motors_des[LeftShoulderPitch] = 1.1
    q_motors_des[LeftShoulderYaw] = 0
    q_motors_des[LeftElbow] = -0.145
    q_motors_des[RightShoulderRoll] = 0.15
    q_motors_des[RightShoulderPitch] = -1.1
    q_motors_des[RightShoulderYaw] = 0
    q_motors_des[RightElbow] = 0.145

    com_midpoint_error = p_com_aligned - 0.5 * 
                (p_left_toe_aligned + p_right_toe_aligned)
    toe_pitch_error = com_midpoint_error[1] 
    k_toe = 1.3
    q_motors_des[LeftToeA] =   q_motors[LeftToeA]  + k_toe*toe_pitch_error
    q_motors_des[LeftToeB] = q_motors[LeftToeB] - k_toe*toe_pitch_error
    q_motors_des[RightToeA] =  q_motors[RightToeA] - k_toe*toe_pitch_error
    q_motors_des[RightToeB] =  q_motors[RightToeB]+ k_toe*toe_pitch_error  

    τ = zeros(20) 
    Δq_motors = q_motors - q_motors_des   

    kp_hiproll_stand = 80  
    kp_hipyaw_stand = 40.0
    kp_hippitch_stand = 50.0
    kp_knee_stand = 200.0 
    kp_knee_comp_stand = 270
    kd_knee_comp_stand = 30

    kp_shoulderroll_stand = 10.0
    kp_shoulderpitch_stand = 10.0
    kp_shoulderyaw_stand = 10.0
    kp_elbow_stand = 10.0 

    τ[LeftHipRoll] = -kp_hiproll_stand * Δq_motors[LeftHipRoll]
    τ[LeftHipYaw] = -kp_hipyaw_stand * Δq_motors[LeftHipYaw]
    τ[LeftHipPitch] = -kp_hippitch_stand * Δq_motors[LeftHipPitch]
    τ[LeftKnee] = -kp_knee_stand * Δq_motors[LeftKnee]
    τ[RightHipRoll] = -kp_hiproll_stand * Δq_motors[RightHipRoll]
    τ[RightHipYaw] = -kp_hipyaw_stand * Δq_motors[RightHipYaw]
    τ[RightHipPitch] = -kp_hippitch_stand * Δq_motors[RightHipPitch]
    τ[RightKnee] = -kp_knee_stand * Δq_motors[RightKnee]

    τ[LeftShoulderRoll] = -kp_shoulderroll_stand * Δq_motors[LeftShoulderRoll]
    τ[LeftShoulderPitch] = -kp_shoulderpitch_stand * Δq_motors[LeftShoulderPitch]
    τ[LeftShoulderYaw] = -kp_shoulderyaw_stand * Δq_motors[LeftShoulderYaw]
    τ[LeftElbow] = -kp_elbow_stand * Δq_motors[LeftElbow]
    τ[RightShoulderRoll] = -kp_shoulderroll_stand * Δq_motors[RightShoulderRoll]
    τ[RightShoulderPitch] = -kp_shoulderpitch_stand * Δq_motors[RightShoulderPitch]
    τ[RightShoulderYaw] = -kp_shoulderyaw_stand * Δq_motors[RightShoulderYaw]
    τ[RightElbow] = -kp_elbow_stand * Δq_motors[RightElbow]

    kp_toe_stand = 10.0  
    kd_toe_stand = 5.0
    τ[LeftToeA] = -kp_toe_stand * Δq_motors[LeftToeA] + kd_toe_stand * v_com_aligned[1]
    τ[LeftToeB] = -kp_toe_stand * Δq_motors[LeftToeB] - kd_toe_stand * v_com_aligned[1]
    τ[RightToeA] = -kp_toe_stand * Δq_motors[RightToeA] - kd_toe_stand * v_com_aligned[1]
    τ[RightToeB] = -kp_toe_stand * Δq_motors[RightToeB] + kd_toe_stand * v_com_aligned[1]

    # knee_comp
    knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    τ[LeftKnee] += knee_comp
    τ[RightKnee] += knee_comp  
    return τ
end
