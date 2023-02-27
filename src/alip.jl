function update_walking_observation(q, qdot, qmotors,  params, problem)
    params[:t] = problem.t
    params[:q] = q
    params[:qdot] = qdot
    params[:qmotors] = qmotors
    params[:leftHeelSpring] = pyconvert(Float64, problem.digit.data.joint("left-heel-spring").qpos[0])
    params[:rightHeelSpring] = pyconvert(Float64, problem.digit.data.joint("right-heel-spring").qpos[0])
    params[:s] = (params[:t] - params[:t0])/params[:swing_time]  
end


function init_params(params)
    if !params[:inited] 
        # params[:target_heading] = params[:q][qbase_yaw]
        params[:inited] = true
    end
end


function made_impact(params)
    deflection = params[:swing_foot] == :left ? params[:leftHeelSpring] : params[:rightHeelSpring]
    if params[:s] > 1.0 || (params[:s] > 0.5 && abs(deflection) > 0.012)
        return true
    end
    return false
end


function transition_to_stand(params)
    if params[:s] > 0.9 #&& params[:swing_foot] == :right
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
            st_heading_bos = wrap_to_pi(params[:q][qbase_yaw] - params[:q][qleftHipYaw])
            params[:Rz_st] = RotZ(st_heading_bos)
            params[:p_sw_wrt_st_toe_bos] = params[:Rz_st]' * (right_toe - left_toe)
            p_com_w = kin.p_COM(params[:q])
            params[:p_com_wrt_st_bos] = params[:Rz_st]' * (p_com_w - left_toe)
            
            params[:indices].idx_q_st_hiproll_ = qleftHipRoll
            params[:indices].idx_q_st_hipyaw_ = qleftHipYaw
            params[:indices].idx_q_st_hippitch_  = qleftHipPitch
            params[:indices].idx_q_st_knee_  = qleftKnee
            params[:indices].idx_q_st_KneeToShin_  = qleftKneeToShin
            params[:indices].idx_q_st_ShinToTarsus_  = qleftShinToTarsus

            params[:indices].idx_q_sw_hiproll_  = qrightHipRoll
            params[:indices].idx_q_sw_hipyaw_  = qrightHipYaw
            params[:indices].idx_q_sw_hippitch_  = qrightHipPitch
            params[:indices].idx_q_sw_knee_  = qrightKnee
            params[:indices].idx_q_sw_KneeToShin_  = qrightKneeToShin
            params[:indices].idx_q_sw_ShinToTarsus_  = qrightShinToTarsus
        
            params[:indices].idx_m_st_hiproll_  = LeftHipRoll
            params[:indices].idx_m_st_hipyaw_ = LeftHipYaw
            params[:indices].idx_m_st_hippitch_  = LeftHipPitch
            params[:indices].idx_m_st_knee_  = LeftKnee

            params[:indices].idx_m_sw_hiproll_  = RightHipRoll
            params[:indices].idx_m_sw_hipyaw_  = RightHipYaw
            params[:indices].idx_m_sw_hippitch_  = RightHipPitch
            params[:indices].idx_m_sw_knee_  = RightKnee
        else
            st_heading_bos = wrap_to_pi(params[:q][qbase_yaw] - params[:q][qrightHipYaw])
            params[:Rz_st] = RotZ(st_heading_bos)
            params[:p_sw_wrt_st_toe_bos] = params[:Rz_st]' * (left_toe - right_toe)
            p_com_w = kin.p_COM(params[:q])
            params[:p_com_wrt_st_bos] = params[:Rz_st]' * (p_com_w - right_toe)

            params[:indices].idx_q_st_hiproll_ = qrightHipRoll
            params[:indices].idx_q_st_hipyaw_ = qrightHipYaw
            params[:indices].idx_q_st_hippitch_  = qrightHipPitch
            params[:indices].idx_q_st_knee_  = qrightKnee
            params[:indices].idx_q_st_KneeToShin_  = qrightKneeToShin
            params[:indices].idx_q_st_ShinToTarsus_  = qrightShinToTarsus

            params[:indices].idx_q_sw_hiproll_  = qleftHipRoll
            params[:indices].idx_q_sw_hipyaw_  = qleftHipYaw
            params[:indices].idx_q_sw_hippitch_  = qleftHipPitch
            params[:indices].idx_q_sw_knee_  = qleftKnee
            params[:indices].idx_q_sw_KneeToShin_  = qleftKneeToShin
            params[:indices].idx_q_sw_ShinToTarsus_  = qleftShinToTarsus
        
            params[:indices].idx_m_st_hiproll_  = RightHipRoll
            params[:indices].idx_m_st_hipyaw_ = RightHipYaw
            params[:indices].idx_m_st_hippitch_  = RightHipPitch
            params[:indices].idx_m_st_knee_  = RightKnee

            params[:indices].idx_m_sw_hiproll_  = LeftHipRoll
            params[:indices].idx_m_sw_hipyaw_  = LeftHipYaw
            params[:indices].idx_m_sw_hippitch_  = LeftHipPitch
            params[:indices].idx_m_sw_knee_  = LeftKnee
        end
        
    end
end


function compute_alip_foot_placement(params)
    com_wrt_feet = kin.p_com_wrt_feet(params[:q])
    id = params[:swing_foot] == :left ? 6 : 3
    zH = com_wrt_feet[id]
    params[:walk_height] = zH 
    lip_constant = sqrt(9.806/zH)
    Ts = params[:swing_time]
    T_r = (1-params[:s])*Ts
    id = params[:swing_foot] == :left ? 4 : 1
    vel_des_st = params[:vel_des_target]
    mass = 46.2104
    step_width = params[:step_width] 

    p_com_w = kin.p_COM(params[:q])
    v_com_w = kin.v_COM(params[:q], params[:qdot])
    if params[:swing_foot] == :right
        p_left_toe_w = kin.p_toe_pitch_joint_left(params[:q])
        p_com_wrt_st_aligned = params[:Rz_st]' * (p_com_w - p_left_toe_w)
        v_com_wrt_st_aligned = params[:Rz_st]' * v_com_w  

        L = kin.angular_momentum_about_point(params[:q], params[:qdot], p_left_toe_w)
        L_aligned = params[:Rz_st]' * L 
        Lx_st = L_aligned[1]
        Ly_st = L_aligned[2]
    else
        p_right_toe_w = kin.p_toe_pitch_joint_right(params[:q])
        p_com_wrt_st_aligned = params[:Rz_st]' * (p_com_w - p_right_toe_w)
        v_com_wrt_st_aligned = params[:Rz_st]' * v_com_w  

        L = kin.angular_momentum_about_point(params[:q], params[:qdot], p_right_toe_w)
        L_aligned = params[:Rz_st]' * L
        Lx_st = L_aligned[1]
        Ly_st = L_aligned[2]
    end
    xc = p_com_wrt_st_aligned[1] 
    yc = p_com_wrt_st_aligned[2]
    vz = v_com_wrt_st_aligned[3] 

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

    p_com_wrt_sw_eos_x = (Ly_des - cosh(lip_constant*Ts)*Ly_eos_est - mass*cosh(lip_constant*Ts) * xc*vz)  /  (mass*zH*lip_constant*sinh(lip_constant*Ts) - mass * cosh(lip_constant*Ts) * vz)
    p_com_wrt_sw_eos_y = (Lx_des - cosh(lip_constant*Ts)*Lx_eos_est + mass*cosh(lip_constant*Ts) * yc*vz)  /  (mass*-zH*lip_constant*sinh(lip_constant*Ts) + mass * cosh(lip_constant*Ts) * vz)

    # p_com_wrt_sw_eos_x = (Ly_des - cosh(lip_constant*Ts)*Ly_eos_est) / (mass*zH*lip_constant*sinh(lip_constant*Ts))
    # p_com_wrt_sw_eos_y = -(Lx_des - cosh(lip_constant*Ts)*Lx_eos_est) / (mass*zH*lip_constant*sinh(lip_constant*Ts))


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