
## Task Maps
# Level 4
function mobile_manipulation_task_map(θ, θ̇ , qmotors,  prob)
    params = prob.task_data[:mm]
    if !(params[:action_index] > length(params[:plan]))
        current_action = params[:plan][params[:action_index]]
        name = current_action[1]
        println("")
        @show name, params[:action_index]
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
                # prob.task_data[name][:start_time] = prob.t
                params[:init_start_time] = true 
            end

        elseif name == :walk_in_place  
            prob.task_data[:mm][:standing] = false
            prob.task_data[name][:period] = current_action[2]
            # activate
            activate_fabric!(name, prob, 3)
            activate_fabric!(:walk, prob, 2)
            activate_fabric!(:walk_attractor, prob, 1)
            activate_fabric!(:upper_body_posture, prob, 1)

            # deactivate
            delete_fabric!(:bimanual_pickup, prob, 3) 
            delete_fabric!(:com_target, prob, 1) 
            if !prob.task_data[name][:init_start_time]
                println("initing")
                prob.task_data[name][:start_time] = prob.t
                prob.task_data[name][:init_start_time] = true 
            end

        elseif name == :precise_move
            prob.task_data[name][:direction] = current_action[2]
            prob.task_data[name][:distance] = current_action[3]
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
            delete_fabric!(:precise_move, prob, 3)
            delete_fabric!(:walk_in_place, prob, 3)
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
            delete_fabric!(:precise_move, prob, 3)
            delete_fabric!(:walk_in_place, prob, 3)
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
            delete_fabric!(:precise_move, prob, 3)
            delete_fabric!(:walk_in_place, prob, 3)
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
            delete_fabric!(:precise_move, prob, 3)
            delete_fabric!(:walk_in_place, prob, 3)
            delete_fabric!(:walk_attractor, prob, 1) 
        end
    end
end

# Level 3
function navigate_task_map(θ, θ̇ , qmotors,  prob)
    params = prob.task_data[:navigate]
    state = params[:state]
    # @show state
    u = [0,0,0.]
    if state == :translate        
        ψ = reach_map 
        p = get_floating_pose(θ, params)
        # @show p
        x = ψ(p, params)
        J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, params), p)
        f = reach_force(x, params)
        u = J'*f  
        delta = norm(p-params[:goal])
        # @show delta 
        if delta < params[:tolerance]
            params[:state] = :stand
            params[:stand_start_time] = prob.t
        end
    elseif state == :stand 
        if prob.task_data[:mm][:action_index] >= length(prob.task_data[:mm][:plan])  switch = true else
            switch = prob.task_data[:mm][:plan][prob.task_data[:mm][:action_index]+1][1] != :navigate 
        end
        if  switch
            s = (prob.t - params[:stand_start_time])/params[:stand_period]
            prob.task_data[:walk][:vel_des_target] = [0,0,0.] 
            if s >= 1.0  && transition_to_stand(prob.task_data[:walk])  
                prob.xᵨ[:com_target][[3,6]] .= prob.task_data[:walk][:walk_height]
                activate_fabric!(:com_target, prob, 1)
                delete_fabric!(:walk_attractor, prob, 1)
                prob.task_data[:mm][:standing] = true
                params[:state] = :translate 
                prob.task_data[:mm][:action_index] += 1  
                prob.task_data[:bimanual_pickup][:action_start_time] = prob.t
            end 
        else 
            params[:state] = :translate 
            prob.task_data[:mm][:action_index] += 1
        end
    end
    prob.task_data[:walk][:vel_des_target] = u
end

function walk_in_place_task_map(θ, θ̇ , qmotors,  prob)
    params = prob.task_data[:walk_in_place]
    state = params[:state]
    u = [0., 0, 0]
    # @show state
    
    # if state == :init 
    #     params[:state] = :walk

    if state == :walk
        s = (prob.t - params[:start_time])/params[:period]
        # @show s
        if s >= 1.0
            params[:state] = :finish
        end

    elseif state == :finish
        prob.task_data[:mm][:action_index] += 1 
        # params[:state] = :walk
    end
    prob.task_data[:walk][:vel_des_target] = u
end


function precise_move_task_map(θ, θ̇ , qmotors,  prob)
    params = prob.task_data[:precise_move]
    state = params[:state]
    @show state
    u = [0., 0, 0]
    if state == :init
        params[:init_position] = θ[[di.qbase_pos_x, di.qbase_pos_y]]
        params[:state] = :walk_in_place
        params[:w_start_time] = prob.t

    elseif state == :walk_in_place
        period = params[:direction] == :forward ? params[:wx_period] : params[:wy_period]
        s = (prob.t - params[:w_start_time])/period
        prob.task_data[:walk][:vel_des_target] = [0,0,0.] 
        if s >= 1.0
            params[:state] = :move
        end
    
    elseif state == :move
        current_pose = θ[[di.qbase_pos_x, di.qbase_pos_y]]
        d = norm(current_pose-params[:init_position])
        sign = params[:distance] < 0 ? -1 : 1
        e = sign*(d - abs(params[:distance]))
        # @show e
        dedt = e/params[:dt]
        v = -params[:Kp]*e - params[:Kd]*dedt
        lim = params[:direction] == :forward ? params[:limx] : params[:limy]
        v = clamp(v, -lim, lim)
        if params[:direction] == :forward u[1] = v else u[2] = v  end
        if abs(e) < params[:tolerance] 
            params[:state] = :purgatory
            params[:stand_start_time] = prob.t
        end

    elseif state == :purgatory
        s = (prob.t - params[:stand_start_time])/params[:stand_period]
        prob.task_data[:walk][:vel_des_target] = [0,0,0.] 
        if s >= 1.0
            params[:state] = :stand
        end


    elseif state == :stand 
        if prob.task_data[:mm][:action_index] >= length(prob.task_data[:mm][:plan])  switch = true else
            switch = !(prob.task_data[:mm][:plan][prob.task_data[:mm][:action_index]+1][1] in (:precise_move, :navigate))
        end
        if  switch
            s = (prob.t - params[:stand_start_time])/params[:stand_period]
            prob.task_data[:walk][:vel_des_target] = [0,0,0.] 
            if s >= 1.0  && transition_to_stand(prob.task_data[:walk])  
                # prob.xᵨ[:com_target][[3,6]] .= prob.task_data[:walk][:walk_height]
                activate_fabric!(:com_target, prob, 1)
                delete_fabric!(:walk_attractor, prob, 1)
                prob.task_data[:mm][:standing] = true
                params[:state] = :init 
                prob.task_data[:mm][:action_index] += 1  
                prob.task_data[:bimanual_pickup][:action_start_time] = prob.t
            end 
        else 
            params[:state] = :init 
            prob.task_data[:mm][:action_index] += 1
        end
    end
    # @show u
    prob.task_data[:walk][:vel_des_target] = u     
end

function bimanual_pickup_task_map(q, qdot, qmotors,  prob)  
    params = prob.task_data[:bimanual_pickup]
    # println("state: $(params[:state])")

    if params[:state] == :descend_init
        t0 = prob.t
        T = params[:flight_time]
        p0 = 0.0; z0 = 0.95#kin.p_com_wrt_feet(q)[3]
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
        # prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
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
        # prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
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

function bimanual_place_task_map(q, qdot, qmotors,  prob) 
    params = prob.task_data[:bimanual_place]
    # println("state: $(params[:state])")

    if params[:state] == :descend_init
        t0 = prob.t 
        T = params[:flight_time]
        p0 = 0.0; z0 = 0.92#kin.p_com_wrt_feet(q)[3]
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
        # prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
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
        # prob.xᵨ[:com_target][[3, 6]] .= height_traj(prob.t)
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
function walk_task_map(q, qdot, qmotors,  problem)
    params = problem.task_data[:walk]
    update_walking_observation(q, qdot, qmotors,  params, problem)
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

function stand_task_map(q, qdot, qmotors,  prob)
    params = prob.task_data[:stand]
    state = params[:state]
    if state == :init
        params[:start_time] = prob.t
        params[:state] = :run
        p0 = q[di.qbase_pitch]; r0 = q[di.qbase_roll]
        params[:pitch_trajectory] = s->(1-s)*p0 + s*params[:torso_pitch]
        params[:roll_trajectory] = s->(1-s)*p0 + s*params[:torso_roll]
    
    elseif state == :run
        s = (prob.t - params[:start_time])/params[:period]
        prob.xᵨ[:com_target][[3,6]] .= params[:com_height]
        prob.xᵨ[:com_target][7] = params[:pitch_trajectory](s)
        prob.xᵨ[:com_target][8] = params[:roll_trajectory](s)
        if s >= 1.0
            params[:state] = :init
            prob.task_data[:mm][:action_index] += 1
            delete_fabric!(:stand, prob, 2)
            println("done standing")
        end
    end       

end

function cornhole_task_map(q, qdot, qmotors,  prob)
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
function walk_attractor_task_map(q, qdot, prob::FabricProblem)
    params = prob.task_data[:walk]
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

function lower_body_posture_task_map(θ, θ̇ , prob::FabricProblem)
    return θ[prob.digit.leg_joint_indices]
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

function dodge_task_map(θ, θ̇ , prob::FabricProblem) 
    θ[di.qleftShinToTarsus] = -θ[di.qleftKnee]
    θ[di.qrightShinToTarsus] = -θ[di.qrightKnee]
    com =  kin.p_base_wrt_feet(θ) 
    com[[3, 6]] .+= 0.5
    # com[[1, 4]] .+= 0.2
    pose = [sum(com[[1,4]])/2, sum(com[[2, 5]])/2, sum(com[[3,6]])/2]
    R = RotZYX([θ[di.qbase_yaw], θ[di.qbase_pitch], θ[di.qbase_roll]]...)
    pose = R*pose
    obs_pose = prob.task_data[:obstacle][:position]
    radius = prob.task_data[:obstacle][:radius]
    Δ = [(norm(pose-obs_pose)/radius) - 1]
    return Δ
end

function zmp_upper_limit_task_map(θ, θ̇ , prob::FabricProblem) 
    params = prob.task_data[:zmp]
    Kfilter = params[:filter]
    vprev = params[:prev_com_vel]
    g = params[:g]
    p_com = kin.p_COM(θ)
    v_com = kin.v_COM(θ, θ̇ )
    h = p_com[3]
    a_com = ((v_com[1:2] - vprev)/(5e-4)) 
    a_com = (1-0.01)*params[:prev_a] + 0.01*a_com
    pz = p_com[1:2] - (h/g)*a_com
    params[:prev_com_vel] = v_com[1:2]
    params[:prev_a] = a_com
    return [params[:upper_limit] - pz[1]]
end

function zmp_lower_limit_task_map(θ, θ̇ , prob::FabricProblem) 
    params = prob.task_data[:zmp]
    Kfilter = params[:filter]
    vprev = params[:prev_com_vel]
    g = params[:g]
    p_com = kin.p_COM(θ)
    v_com = kin.v_COM(θ, θ̇ )
    h = p_com[3]
    a_com = ((v_com[1:2] - vprev)/(5e-4)) 
    a_com = (1-0.01)*params[:prev_a] + 0.01*a_com
    pz = p_com[1:2] - (h/g)*a_com
    params[:prev_com_vel] = v_com[1:2]
    params[:prev_a] = a_com
    return [pz[1] - params[:lower_limit]]
end

function joint_lower_limit_task_map(θ, θ̇ , prob::FabricProblem)
    return θ[prob.digit.arm_joint_indices]-prob.digit.θ_min[prob.digit.arm_joint_indices]
end

function joint_upper_limit_task_map(θ, θ̇ , prob::FabricProblem)
    return prob.digit.θ_max[prob.digit.arm_joint_indices] - θ[prob.digit.arm_joint_indices]
end

function left_hand_target_task_map(θ, θ̇ , prob::FabricProblem)  
    θ[di.qleftShinToTarsus] = -θ[di.qleftKnee]
    θ[di.qrightShinToTarsus] = -θ[di.qrightKnee]
    pos =  kin.p_left_hand_wrt_feet_center(θ) 
    res = pos 
    return res
end

function right_hand_target_task_map(θ, θ̇ , prob::FabricProblem) 
    θ[di.qleftShinToTarsus] = -θ[di.qleftKnee]
    θ[di.qrightShinToTarsus] = -θ[di.qrightKnee]
    pos =  kin.p_right_hand_wrt_feet_center(θ) 
    res = pos 
    return res
end
