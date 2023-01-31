# helpers
function update_walking_observation(q, qdot, qmotors, observation, params, problem)
    params[:t] = problem.t
    params[:q] = q
    params[:qdot] = qdot
    params[:qmotors] = collect(qmotors)
    params[:leftHeelSpring] = observation.joint.position[5]
    params[:rightHeelSpring] = observation.joint.position[10]
    params[:s] = (params[:t] - params[:t0])/params[:swing_time]
end

function init_params(params)
    if !params[:inited] 
        params[:target_heading] = params[:q][di.qbase_yaw]
        params[:inited] = true
    end
end

function made_impact(params)
    deflection = params[:swing_foot] == :left ? params[:leftHeelSpring] : params[:rightHeelSpring]
    if params[:s] > 1.0 || (params[:s] > 0.5 && abs(deflection) > 0.015)
        return true
    end
    return false
end

function check_stance_switch(params)
    if made_impact(params)
        params[:swing_foot] = params[:swing_foot] == :left ? :right : :left 
        params[:t0] = params[:t]
        left_toe = kin.p_toe_pitch_joint_left(params[:q])
        right_toe = kin.p_toe_pitch_joint_right(params[:q])
        params[:target_heading] = wrap_to_pi(params[:target_heading] + params[:vel_des_target][3]*params[:swing_time])
        if params[:swing_foot] == :right
            st_heading_bos = wrap_to_pi(params[:q][di.qbase_yaw] - params[:q][di.qleftHipYaw])
            params[:Rz_st] = RotZ(st_heading_bos)
            params[:p_sw_wrt_st_toe_bos] = params[:Rz_st]' * (right_toe - left_toe)
            p_com_w = kin.p_COM(params[:q])
            params[:p_com_wrt_st_bos] = params[:Rz_st]' * (p_com_w - left_toe)
            
            params[:indices].idx_q_st_hiproll_ = di.qleftHipRoll
            params[:indices].idx_q_st_hipyaw_ = di.qleftHipYaw
            params[:indices].idx_q_st_hippitch_  = di.qleftHipPitch
            params[:indices].idx_q_st_knee_  = di.qleftKnee
            params[:indices].idx_q_st_KneeToShin_  = di.qleftKneeToShin
            params[:indices].idx_q_st_ShinToTarsus_  = di.qleftShinToTarsus

            params[:indices].idx_q_sw_hiproll_  = di.qrightHipRoll
            params[:indices].idx_q_sw_hipyaw_  = di.qrightHipYaw
            params[:indices].idx_q_sw_hippitch_  = di.qrightHipPitch
            params[:indices].idx_q_sw_knee_  = di.qrightKnee
            params[:indices].idx_q_sw_KneeToShin_  = di.qrightKneeToShin
            params[:indices].idx_q_sw_ShinToTarsus_  = di.qrightShinToTarsus
        
            params[:indices].idx_m_st_hiproll_  = di.LeftHipRoll
            params[:indices].idx_m_st_hipyaw_ = di.LeftHipYaw
            params[:indices].idx_m_st_hippitch_  = di.LeftHipPitch
            params[:indices].idx_m_st_knee_  = di.LeftKnee

            params[:indices].idx_m_sw_hiproll_  = di.RightHipRoll
            params[:indices].idx_m_sw_hipyaw_  = di.RightHipYaw
            params[:indices].idx_m_sw_hippitch_  = di.RightHipPitch
            params[:indices].idx_m_sw_knee_  = di.RightKnee
        else
            st_heading_bos = wrap_to_pi(params[:q][di.qbase_yaw] - params[:q][di.qrightHipYaw])
            params[:Rz_st] = RotZ(st_heading_bos)
            params[:p_sw_wrt_st_toe_bos] = params[:Rz_st]' * (left_toe - right_toe)
            p_com_w = kin.p_COM(params[:q])
            params[:p_com_wrt_st_bos] = params[:Rz_st]' * (p_com_w - right_toe)

            params[:indices].idx_q_st_hiproll_ = di.qrightHipRoll
            params[:indices].idx_q_st_hipyaw_ = di.qrightHipYaw
            params[:indices].idx_q_st_hippitch_  = di.qrightHipPitch
            params[:indices].idx_q_st_knee_  = di.qrightKnee
            params[:indices].idx_q_st_KneeToShin_  = di.qrightKneeToShin
            params[:indices].idx_q_st_ShinToTarsus_  = di.qrightShinToTarsus

            params[:indices].idx_q_sw_hiproll_  = di.qleftHipRoll
            params[:indices].idx_q_sw_hipyaw_  = di.qleftHipYaw
            params[:indices].idx_q_sw_hippitch_  = di.qleftHipPitch
            params[:indices].idx_q_sw_knee_  = di.qleftKnee
            params[:indices].idx_q_sw_KneeToShin_  = di.qleftKneeToShin
            params[:indices].idx_q_sw_ShinToTarsus_  = di.qleftShinToTarsus
        
            params[:indices].idx_m_st_hiproll_  = di.RightHipRoll
            params[:indices].idx_m_st_hipyaw_ = di.RightHipYaw
            params[:indices].idx_m_st_hippitch_  = di.RightHipPitch
            params[:indices].idx_m_st_knee_  = di.RightKnee

            params[:indices].idx_m_sw_hiproll_  = di.LeftHipRoll
            params[:indices].idx_m_sw_hipyaw_  = di.LeftHipYaw
            params[:indices].idx_m_sw_hippitch_  = di.LeftHipPitch
            params[:indices].idx_m_sw_knee_  = di.LeftKnee
        end
        
    end
end

function compute_alip_foot_placement(params)
    com_wrt_feet = kin.p_com_wrt_feet(params[:q])
    id = params[:swing_foot] == :left ? 6 : 3
    zH = com_wrt_feet[id]
    lip_constant = sqrt(9.806/zH)
    Ts = params[:swing_time]
    T_r = (1-params[:s])*Ts
    id = params[:swing_foot] == :left ? 4 : 1
    vel_des_st = params[:vel_des_target]
    mass = 46.2104
    step_width = params[:step_width]

    p_com_w = kin.p_COM(params[:q])
    if params[:swing_foot] == :right
        p_left_toe_w = kin.p_toe_pitch_joint_left(params[:q])
        p_com_wrt_st_aligned = params[:Rz_st]' * (p_com_w - p_left_toe_w)
        L = kin.angular_momentum_about_point(params[:q], params[:qdot], p_left_toe_w)
        L_aligned = params[:Rz_st]' * L 
        Lx_st = L_aligned[1]
        Ly_st = L_aligned[2]
    else
        p_right_toe_w = kin.p_toe_pitch_joint_right(params[:q])
        p_com_wrt_st_aligned = params[:Rz_st]' * (p_com_w - p_right_toe_w)
        L = kin.angular_momentum_about_point(params[:q], params[:qdot], p_right_toe_w)
        L_aligned = params[:Rz_st]' * L
        Lx_st = L_aligned[1]
        Ly_st = L_aligned[2]
    end
    xc = p_com_wrt_st_aligned[1] 
    yc = p_com_wrt_st_aligned[2]


    if params[:swing_foot] == :right  
        Lx_des = (-mass*zH*vel_des_st[2]) - 0.5*mass*zH*step_width*
            ((lip_constant*sinh(lip_constant*Ts)) / (1+cosh(lip_constant*Ts))) 
    else 
        Lx_des = (-mass*zH*vel_des_st[2]) + 0.5*mass*zH*step_width*
        ((lip_constant*sinh(lip_constant*Ts)) / (1+cosh(lip_constant*Ts))) 
    end
    Ly_des = mass*zH*vel_des_st[1]

    xc_eos_est = cosh(lip_constant * T_r) * xc + (1 / (mass*zH*lip_constant)) * sinh(lip_constant*T_r) * Ly_st
    yc_eos_est = cosh(lip_constant * T_r) * yc - (1 / (mass*zH*lip_constant)) * sinh(lip_constant*T_r) * Lx_st

    Lx_eos_est = -1*mass*zH*lip_constant*sinh(lip_constant*T_r)*yc + cosh(lip_constant*T_r) * Lx_st
    Ly_eos_est = mass*zH*lip_constant*sinh(lip_constant*T_r)*xc + cosh(lip_constant*T_r)*Ly_st 

    p_com_wrt_sw_eos_x = (Ly_des - cosh(lip_constant*Ts)*Ly_eos_est) / (mass*zH*lip_constant*sinh(lip_constant*Ts))
    p_com_wrt_sw_eos_y = -(Lx_des - cosh(lip_constant*Ts)*Lx_eos_est) / (mass*zH*lip_constant*sinh(lip_constant*Ts))

    p_sw_wrt_st_eos_x = xc_eos_est - p_com_wrt_sw_eos_x
    p_sw_wrt_st_eos_y = yc_eos_est - p_com_wrt_sw_eos_y
    p_sw_wrt_st_eos_z = 0.0

    foot_placement = [p_sw_wrt_st_eos_x, p_sw_wrt_st_eos_y, p_sw_wrt_st_eos_z]
    max_x_step = 0.7
    foot_placement[1] = max(min(foot_placement[1], max_x_step), -max_x_step)

    max_y_step = 0.6
    min_y_step = 0.1
    sn = params[:swing_foot] == :right ? -1 : 1
    foot_placement[2] = sn * max(min(abs(foot_placement[2]), max_y_step), min_y_step)
    
    return foot_placement
end

function compute_virtual_constraint(foot_placement::AbstractArray, params::Dict)
    bos = params[:p_sw_wrt_st_toe_bos]; Δz = 0.1; Δs = 0.5; ṡ = 1.0/params[:swing_time]
    vc_x = 0.5*(bos[1] - foot_placement[1]) * cos(π*params[:s]) + 0.5*(bos[1]+foot_placement[1])
    vc_y = 0.5*(bos[2] - foot_placement[2]) * cos(π*params[:s]) + 0.5*(bos[2]+foot_placement[2])
    
    vc_dot_x = -0.5*π*ṡ*(bos[1]-foot_placement[1])*sin(π*params[:s])
    vc_dot_y = -0.5*π*ṡ*(bos[2]-foot_placement[2])*sin(π*params[:s])

    α₂ = Δz/(Δs-1) - (Δz-bos[3])/Δs 
    α₁ = -(Δz - bos[3]+bos[3]*Δs*Δs)/(Δs*(Δs-1))
    α₀ = bos[3]
    vc_z = α₂*params[:s]*params[:s] + α₁*params[:s] + α₀
    vc_dot_z = (2*params[:s]*α₂ + α₁)*ṡ

    id = params[:swing_foot] == :left ? 6 : 3
    vc_H = kin.p_com_wrt_feet(params[:q])[id]
    vc = [vc_x, vc_y, vc_z, 0.92]
    vcdot = [vc_dot_x, vc_dot_y, vc_dot_z, 0.0]
    return vc, vcdot
end

function rotate_vc_to_world(vc, vcdot, params)
    p_sw_wrt_st_eos_des_w = params[:Rz_st] * vc[1:3]
    pdot_sw_wrt_st_eos_des_w = params[:Rz_st] * vcdot[1:3]
    vc_w = [p_sw_wrt_st_eos_des_w; vc[4]]
    vcdot_w = [pdot_sw_wrt_st_eos_des_w; vcdot[4]]

    return vc_w, vcdot_w
end

function attractor_task_map(x, params::Dict)
    res = x - params[:goal] 
    return res 
end

function attractor_potential(x, params::Dict)
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
## Task Maps

# Level 4
function mobile_manipulation_task_map(θ, θ̇ , qmotors, observation, prob)
    params = prob.task_data[:mm]
    if !(params[:action_index] > length(params[:plan]))
        current_action = params[:plan][params[:action_index]]
        name = current_action[1]
        println("")
        # @show "current action: $name"
        if name == :navigate 
            prob.task_data[name][:goal] = current_action[2] 
            prob.task_data[:mm][:standing] = false
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:walk, prob, 2)
            activate_fabric!(:walk_attractor, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            # deactivate
            delete_fabric!(:bimanual_pickup, prob, 3) 
            delete_fabric!(:com_target, prob, 1)
            if !prob.task_data[name][:init_start_time]
                prob.task_data[name][:start_time] = prob.t
                params[:init_start_time] = true 
            end

        elseif name == :bimanual_pickup
            prob.task_data[:mm][:standing] = true
            prob.task_data[name][:final_com] = current_action[2]
            prob.task_data[name][:final_pitch] = current_action[3]
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:upper_body_posture, prob, 1)
            activate_fabric!(:com_target, prob, 1)

            # deactivate
            delete_fabric!(:navigate, prob, 3)
            delete_fabric!(:walk, prob, 2)
            delete_fabric!(:walk_attractor, prob, 1)  
            if !prob.task_data[name][:init_start_time]
                prob.task_data[name][:start_time] = prob.t
                prob.task_data[name][:init_start_time] = true 
            end

        elseif name == :bimanual_place
            prob.task_data[:mm][:standing] = true
            prob.task_data[name][:final_com] = current_action[2]
            prob.task_data[name][:final_pitch] = current_action[3]
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:upper_body_posture, prob, 1)
            activate_fabric!(:com_target, prob, 1)

            # deactivate
            delete_fabric!(:navigate, prob, 3)
            delete_fabric!(:walk, prob, 2)
            delete_fabric!(:walk_attractor, prob, 1) 
            if !prob.task_data[name][:init_start_time]
                prob.task_data[name][:start_time] = prob.t
                prob.task_data[name][:init_start_time] = true 
            end
        elseif name == :stand
            prob.task_data[:mm][:standing] = true
            prob.task_data[name][:com_height] = current_action[2]
            prob.task_data[name][:torso_pitch] = current_action[3]
            prob.task_data[name][:period] = current_action[4]
            prob.task_data[name][:torso_roll] = current_action[5]
            
            #activate
            activate_fabric!(name, prob, 2)
            activate_fabric!(:com_target, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            #deactivate
            delete_fabric!(:navigate, prob, 3)
            delete_fabric!(:walk, prob, 2)
            delete_fabric!(:walk_attractor, prob, 1) 
        
        elseif name == :cornhole
            prob.task_data[:mm][:standing] = true
            #activate
            activate_fabric!(name, prob, 2)
            activate_fabric!(:com_target, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            #deactivate
            delete_fabric!(:navigate, prob, 3)
            delete_fabric!(:walk, prob, 2)
            delete_fabric!(:walk_attractor, prob, 1) 
        end
    end
end

# Level 3
function navigate_task_map(θ, θ̇ , qmotors, observation, prob)
    params = prob.task_data[:navigate]
    state = params[:state]
    # @show state
    u = [0,0,0.]
    if state == :translate        
        ψ = attractor_task_map 
        p = get_floating_pose(θ, params)
        @show p
        x = ψ(p, params)
        J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, params), p)
        f = attractor_potential(x, params)
        u = J'*f 
        # u = clamp.(u, -[0.1,0.1,0.1], [0.1,0.1,0.1])
        delta = norm(p-params[:goal])
        @show delta
        # if delta < params[:tolerance]*2
        #     prob.task_data[:walk][:step_width] = 0.4
        # end
        if delta < params[:tolerance]
            params[:state] = :stand
            params[:stand_start_time] = prob.t
        end
    elseif state == :stand 
        if length(prob.task_data[:mm][:plan]) == 1 switch = true else
            switch = prob.task_data[:mm][:plan][prob.task_data[:mm][:action_index]+1][1] != :navigate 
        end
        if  switch
            s = (prob.t - params[:stand_start_time])/params[:stand_period]
            prob.task_data[:walk][:vel_des_target] = [0,0,0.]
            @show prob.task_data[:walk][:s]
            if s >= 1.0  
                activate_fabric!(:com_target, prob, 1)
                delete_fabric!(:walk_attractor, prob, 1)
                prob.task_data[:mm][:standing] = true 
                if made_impact(prob.task_data[:walk])
                    params[:state] = :translate
                    # prob.task_data[:walk][:step_width] = 0.3
                    prob.task_data[:mm][:action_index] += 1  
                    prob.task_data[:bimanual_pickup][:action_start_time] = prob.t
                end
            end
        else
            # @show "translating"
            params[:state] = :translate 
            prob.task_data[:mm][:action_index] += 1
        end
    end
    prob.task_data[:walk][:vel_des_target] = u
end

function bimanual_pickup_task_map(q, qdot, qmotors, observation, prob)  
    params = prob.task_data[:bimanual_pickup]
    println("state: $(params[:state])")

    if params[:state] == :descend_init
        t0 = prob.t
        T = params[:flight_time]
        p0 = 0.0; z0 = 0.95
        pf = params[:final_pitch]; zf = params[:final_com]        
        pitch(t) = (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        com_height(t) = (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf
        params[:pitch_trajectory] = pitch
        params[:com_height_trajectory] = com_height
        s = (prob.t - params[:start_time])/params[:start_buffer_time]
        if s >= 1.0
            params[:state] = :descend
            params[:start_time] = prob.t
        end
    
    elseif params[:state] == :descend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.xᵨ[:com_target][7] = pitch_traj(prob.t) 
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:open_arms_posture]  
        if abs(params[:flight_time] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :clasp
            params[:clasp_start_time] = prob.t
        end 

    elseif params[:state] == :clasp
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:close_arms_posture] 
        s = (prob.t - params[:clasp_start_time])/params[:clasp_period]
        if s >= 1.0
            params[:state] = :ascend_init
        end
        
    elseif params[:state] == :ascend_init
        t0 = prob.t
        params[:start_time] = t0
        T = params[:flight_time]
        p0 = params[:final_pitch]; z0 = params[:final_com]
        pf = 0.0; zf = 0.95
        ascend_pitch(t) = (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        ascend_com_height(t) = (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf
        params[:pitch_trajectory] = ascend_pitch
        params[:com_height_trajectory] = ascend_com_height
        params[:state] = :ascend 

    elseif params[:state] == :ascend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.xᵨ[:com_target][7] = pitch_traj(prob.t) 
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:clutch_arms_posture] 
        if abs(params[:flight_time] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :stand
        end

    elseif params[:state] == :stand 
        params[:state] = :finish
        prob.task_data[:mm][:action_index] += 1
        delete_fabric!(:bimanual_pickup, prob, 3)

    elseif params[:state] == :finish   
        params[:state] = :descend_init
    end

end

function bimanual_place_task_map(q, qdot, qmotors, observation, prob) 
    params = prob.task_data[:bimanual_place]
    println("state: $(params[:state])")

    if params[:state] == :descend_init
        t0 = prob.t 
        T = params[:flight_time]
        p0 = 0.0; z0 = 0.95
        pf = params[:final_pitch]; zf = params[:final_com]        
        pitch(t) = (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        com_height(t) = (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf
        params[:pitch_trajectory] = pitch
        params[:com_height_trajectory] = com_height
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:close_arms_posture]
        s = (prob.t - params[:start_time])/params[:start_buffer_time] 
        if s >= 1.0
            params[:state] = :descend
            params[:start_time] = prob.t
        end
    
    elseif params[:state] == :descend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.xᵨ[:com_target][7] = pitch_traj(prob.t)   
        if abs(params[:flight_time] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :open
            params[:open_start_time] = prob.t
        end 

    elseif params[:state] == :open
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:open_arms_posture] 
        s = (prob.t - params[:open_start_time])/params[:open_period]
        if s >= 1.0
            params[:state] = :ascend_init
        end
        
    elseif params[:state] == :ascend_init
        t0 = prob.t
        params[:start_time] = t0
        T = params[:flight_time]
        p0 = params[:final_pitch]; z0 = params[:final_com]
        pf = 0.0; zf = 0.95
        ascend_pitch(t) = (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        ascend_com_height(t) = (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf
        params[:pitch_trajectory] = ascend_pitch
        params[:com_height_trajectory] = ascend_com_height
        params[:state] = :ascend 

    elseif params[:state] == :ascend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.xᵨ[:com_target][7] = pitch_traj(prob.t) 
        if abs(params[:flight_time] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :stand
        end

    elseif params[:state] == :stand 
        params[:state] = :finish
        prob.task_data[:mm][:action_index] += 1
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:normal_posture]
        delete_fabric!(:bimanual_place, prob, 3)

    elseif params[:state] == :finish  
        params[:state] = :descend_init
    end

end

# Level 2
function walk_task_map(q, qdot, qmotors, observation, problem)
    params = problem.task_data[:walk]
    update_walking_observation(q, qdot, qmotors, observation, params, problem)
    init_params(params)
    check_stance_switch(params)  
    fp = compute_alip_foot_placement(params)
    vc, vcdot = compute_virtual_constraint(fp, params)
    vc, vcdot = rotate_vc_to_world(vc, vcdot, params) 
    s = zeros(length(q))
    leg_indices = [params[:indices].idx_q_sw_hiproll_, params[:indices].idx_q_sw_hippitch_, params[:indices].idx_q_sw_knee_, params[:indices].idx_q_st_knee_]
    s[leg_indices] .= 1.0
    problem.S[:walk_attractor] = diagm(s)
    problem.xᵨ[:walk_attractor] = [vc..., vcdot...] 
end

function stand_task_map(q, qdot, qmotors, observation, prob)
    params = prob.task_data[:stand]
    state = params[:state]
    if state == :init
        params[:start_time] = prob.t
        params[:state] = :run
    
    elseif state == :run
        s = (prob.t - params[:start_time])/params[:period]
        prob.xᵨ[:com_target][[3,6]] .= params[:com_height]
        prob.xᵨ[:com_target][7] = params[:torso_pitch]
        prob.xᵨ[:com_target][8] = params[:torso_roll]
        if s >= 1.0
            params[:state] = :init
            prob.task_data[:mm][:action_index] += 1
            delete_fabric!(:stand, prob, 2)
            println("done standing")
        end
    end       

end

function cornhole_task_map(q, qdot, qmotors, observation, prob)
    params = prob.task_data[:cornhole]
    state = params[:state]
    @show state

    if state == :init
        open_gripper!(params[:gripper])
        println("open gripper")
        params[:start_time] = prob.t
        params[:state] = :descend_init
    
    elseif params[:state] == :descend_init
        t0 = prob.t
        T = params[:descend_period]
        p0 = 0.0; z0 = 0.95
        pf = params[:pick_torso_pitch]; zf = params[:pick_height]    
        params[:pitch_trajectory] = t -> (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        params[:com_height_trajectory] = t -> (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf  
        params[:state] = :descend
        params[:start_time] = prob.t 
    
    elseif params[:state] == :descend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.xᵨ[:com_target][7] = pitch_traj(prob.t)  
        if abs(params[:descend_period] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :pick
            params[:start_time] = prob.t
        end
    
    elseif state == :pick
        s = (prob.t - params[:start_time])/params[:pick_period]
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:pick_posture]
        if s >= 1.0
            params[:state] = :clasp
            params[:start_time] = prob.t
        end
    
    elseif state == :clasp
        s = (prob.t - params[:start_time])/params[:clasp_period]
        close_gripper!(params[:gripper])
        println("close gripper")
        if s >= 1.0
            params[:state] = :ascend_init
            params[:start_time] = prob.t
        end

    elseif params[:state] == :ascend_init
        t0 = prob.t
        T = params[:ascend_period]
        p0 = 0.0; z0 = params[:pick_height]
        pf = params[:throw_torso_pitch]; zf = params[:throw_height]    
        params[:pitch_trajectory] = t -> (1 - ((t-t0)/T))*p0 + ((t-t0)/T)*pf
        params[:com_height_trajectory] = t -> (1 - ((t-t0)/T))*z0 + ((t-t0)/T)*zf 
        params[:state] = :ascend
        params[:start_time] = prob.t 
    
    elseif params[:state] == :ascend
        pitch_traj = params[:pitch_trajectory]
        height_traj = params[:com_height_trajectory] 
        prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
        prob.xᵨ[:com_target][7] = pitch_traj(prob.t)  
        if abs(params[:ascend_period] - (prob.t - params[:start_time])) < 1e-2
            params[:state] = :load
            params[:start_time] = prob.t
        end

    elseif state == :load
        s = (prob.t - params[:start_time])/params[:load_period]
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:load_posture]
        if s >= 1.0
            params[:state] = :throw
            params[:start_time] = prob.t
        end
    
    elseif state == :throw 
        s = (prob.t - params[:start_time])/params[:throw_period]
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:throw_posture]
        params[:fling] = true
        if 0.2<s<0.4 
            open_gripper!(params[:gripper])
            println("open gripper")
        end

        if s >= 1.0
            params[:state] = :finish
            params[:fling] = false
            params[:start_time] = prob.t
        end

    elseif state == :finish 
        prob.xᵨ[:upper_body_posture] = prob.xᵨ[:normal_posture]
    end

end


# Level 1
function walk_attractor_task_map(q, qdot, problem)
    params = problem.task_data[:walk]
    q[params[:indices].idx_q_st_KneeToShin_] = 0.0
    q[params[:indices].idx_q_sw_KneeToShin_] = 0.0
    q[params[:indices].idx_q_st_ShinToTarsus_] = -q[params[:indices].idx_q_st_knee_]
    q[params[:indices].idx_q_sw_ShinToTarsus_] = -q[params[:indices].idx_q_sw_knee_]

    qdot[params[:indices].idx_q_st_KneeToShin_] = 0.0
    qdot[params[:indices].idx_q_sw_KneeToShin_] = 0.0
    qdot[params[:indices].idx_q_st_ShinToTarsus_] = -qdot[params[:indices].idx_q_st_knee_]
    qdot[params[:indices].idx_q_sw_ShinToTarsus_] = -qdot[params[:indices].idx_q_sw_knee_]

    if params[:swing_foot] == :right 
        pos = kin.vc_walk_LS(q)
    else
        pos = kin.vc_walk_RS(q)
    end 
    return pos
end

function upper_body_posture_task_map(θ, θ̇ , prob::FabricProblem)
    return θ[prob.digit.arm_joint_indices]
end

function com_target_task_map(θ, θ̇ , prob::FabricProblem)
    S = prob.S[:com_target]
    θ = S*θ 
    θ[di.qleftShinToTarsus] = -θ[di.qleftKnee]
    θ[di.qrightShinToTarsus] = -θ[di.qrightKnee]
    θ[di.qbase_pitch] = prob.xᵨ[:com_target][7] 
    θ[di.qbase_roll] = prob.xᵨ[:com_target][8] 
    com_pos =  kin.p_com_wrt_feet(θ) 
    Rz = RotZ(θ[di.qbase_yaw]) 
    com = [(Rz * com_pos[1:3])..., (Rz * com_pos[4:6])...]
    res = com
    return res
end


## Fabric Components
function walk_attractor_fabric(x, ẋ, problem)
    k = 1.0; β=0.5; λ = 1.0
    N = length(x)
    W = problem.W[:walk_attractor]
    k = W*k  
    vc_goal = problem.xᵨ[:walk_attractor][1:4]
    Δx = x - vc_goal
    ψ(θ) = 0.5*θ'*k*θ
    δx = FiniteDiff.finite_difference_gradient(ψ, Δx)
    ẍ = -k*δx  
    M = λ * I(N)
    return (M, ẍ)
end

function upper_body_posture_fabric(x, ẋ, prob::FabricProblem) 
    λᵪ = 0.25; k = 4.0;  β=0.75
    M = λᵪ * I(length(x))
    W = prob.W[:upper_body_posture] 
    k = W*k 
    Δx = x - prob.xᵨ[:upper_body_posture]
    ψ(θ) = 0.5*θ'*k*θ
    δx = FiniteDiff.finite_difference_gradient(ψ, Δx)
    ẍ = -k*δx - β*ẋ
    return (M, ẍ) 
end

function com_target_fabric(x, ẋ, prob::FabricProblem; α=40.0, β=40.0, ϵ=0.01)
    k = 5.0; αᵩ = 10.0; β=0.5; λ = 0.25 
    N = length(x)
    W = prob.W[:com_target]
    k = W*k  
    Δx = x - prob.xᵨ[:com_target][1:6]
    ψ(θ) = 0.5*θ'*k*θ
    δx = FiniteDiff.finite_difference_gradient(ψ, Δx)
    ẍ = -k*δx -β*ẋ 
    M = λ * I(N)
    return (M, ẍ)
end


## Solve
function mm_fabric_eval(x, ẋ, name::Symbol, prob::FabricProblem)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, prob)
    return (M, ẍ)
end

function compute_task_map_jacobian(t, ψ, θ, θ̇ , prob::FabricProblem)  
    if t == :walk_attractor
        params = prob.task_data[:walk]
        J = params[:swing_foot] == :right ? kin.Jvc_walk_LS(θ) : kin.Jvc_walk_RS(θ)
        J[:, params[:indices].idx_q_sw_knee_] = J[:, params[:indices].idx_q_sw_knee_] - J[:, params[:indices].idx_q_sw_ShinToTarsus_]
        J[:, params[:indices].idx_q_st_knee_] = J[:, params[:indices].idx_q_st_knee_] - J[:, params[:indices].idx_q_st_ShinToTarsus_]
        J[:, params[:indices].idx_q_st_knee_] = J[:, params[:indices].idx_q_st_knee_] + 0.5*J[:, params[:indices].idx_q_st_hippitch_]
    else
        J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇ , prob), θ) 
    end 
    return J
end

function mm_fabric_solve(θ, θ̇ , qmotors, observation, prob::FabricProblem)
    xₛ = []; ẋₛ = []; cₛ = []; qvel = zero(θ)
    Mₛ = []; ẍₛ = []; Jₛ =  [] 
    for t in prob.ψ[:level4]
        ψ = eval(Symbol(t, :_task_map))
        ψ(θ, θ̇ , qmotors, observation, prob)
    end
    for t in prob.ψ[:level3]
        ψ = eval(Symbol(t, :_task_map))
        ψ(θ, θ̇ , qmotors, observation, prob)
    end
    for t in prob.ψ[:level2]
        ψ = eval(Symbol(t, :_task_map))
        ψ(θ, θ̇ , qmotors, observation, prob)
    end 
    for t in prob.task_data[:mm][:task_maps]
        ψ = eval(Symbol(t, :_task_map))
        S = prob.S[t] 
        J = compute_task_map_jacobian(t, ψ, θ, θ̇ , prob)  
        J = J*S 
        x = ψ(θ, θ̇ , prob)  
        ẋ = J*θ̇ 
        c = zero(x) 
        M, ẍ = mm_fabric_eval(x, ẋ, t, prob)  
        if t == :walk_attractor qvel = J\prob.xᵨ[:walk_attractor][5:end] end
        if t in prob.ψ[:level1]
            push!(xₛ, x); push!(ẋₛ, ẋ); push!(cₛ, c)
            push!(Mₛ, M); push!(ẍₛ, ẍ); push!(Jₛ, J)
        end
    end   
    Mᵣ = sum([J' * M * J for (J, M) in zip(Jₛ, Mₛ)])
    fᵣ = sum([J' * M * (ẍ - c) for (J, M, ẍ, c) in zip(Jₛ, Mₛ, ẍₛ, cₛ)])
    Mᵣ = convert(Matrix{Float64}, Mᵣ)  
    q̈ = pinv(Mᵣ) * fᵣ
    return  q̈, qvel  
end

function mm_fabric_compute(q, qdot, qmotors, observation, problem)
    θ̈d, qvel = mm_fabric_solve(copy(q), copy(qdot), qmotors, observation, problem) 
    θ̇d = θ̈d
    θd = q + θ̇d

    q_out = copy(q)
    qdot_out = copy(qdot)

    if problem.task_data[:mm][:standing]
        θd = q + θ̇d*1e-1
        q_out[problem.digit.leg_joint_indices] = θd[problem.digit.leg_joint_indices]
    else
        params = problem.task_data[:walk]
        indices = [params[:indices].idx_q_sw_hiproll_, params[:indices].idx_q_sw_hippitch_, params[:indices].idx_q_sw_knee_, params[:indices].idx_q_st_knee_]
        q_out[indices] = θd[indices]
        qdot_out[indices] = qvel[indices]  + θ̇d[indices]
    end
    q_out[problem.digit.arm_joint_indices] = θd[problem.digit.arm_joint_indices]
    qdot_out[problem.digit.arm_joint_indices] = θ̇d[problem.digit.arm_joint_indices] 

    
    q_out = clamp.(q_out, problem.digit.θ_min, problem.digit.θ_max)  
    qdot_out = clamp.(qdot_out, problem.digit.θ̇_min, problem.digit.θ̇_max)

    q_out = filter_coordinates(q_out, problem, :pos)
    qdot_out = filter_coordinates(qdot_out, problem, :vel)

    # push!(problem.task_data[:diagnostics][:q], q_out[problem.digit.leg_joint_indices])
    # push!(problem.task_data[:diagnostics][:qdot], qdot_out[problem.digit.leg_joint_indices])
    # push!(problem.task_data[:diagnostics][:t], problem.t)

    τ = zero(q_out) 
    return  q_out, qdot_out, τ
end