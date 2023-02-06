function initialize_solver(N; Δt=1e-3)
    model = JuMP.Model(OSQP.Optimizer) 
    set_silent(model)
    @variable(model, q̇[1:N])
    return model
end

function dodge_qp_task_map(θ, θ̇ , prob::FabricProblem)  
    θ[di.qleftShinToTarsus] = -θ[di.qleftKnee]
    θ[di.qrightShinToTarsus] = -θ[di.qrightKnee]
    com =  kin.p_base_wrt_feet(θ) 
    com[[3, 6]] .+= 0.5
    pose = [0.15+sum(com[[1,4]])/2, sum(com[[2, 5]])/2, sum(com[[3,6]])/2]
    R = RotZYX([θ[di.qbase_yaw], θ[di.qbase_pitch], θ[di.qbase_roll]]...)
    pose = R*pose 
    return pose
end

function build_jacobian(task, θ, θ̇, S, prob)
    ψ = eval(Symbol(task, :_task_map))
    J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇ , prob), θ)
    J = J*S
    return J
end

function get_task_space_coordinate(task, θ, θ̇, prob)
    ψ = eval(Symbol(task, :_task_map))
    x = ψ(θ, θ̇, prob)
    return x
end

function build_attractors(θ, θ̇, prob; K=5.5) 
    Js = []
    vs = []
    ws = [] 
    for task in prob.ψ[:level1] 
        if task != :zmp_upper && task != :zmp_lower && task != :dodge
            S = prob.S[task]
            push!(Js, build_jacobian(task, θ, θ̇, S, prob))
            push!(ws, prob.W[task])
            x = get_task_space_coordinate(task, θ, θ̇, prob)
            target = prob.xᵨ[task]
            e = (target - x)
            push!(vs, K*e)
        end
    end
    return Js, vs, ws
end

function build_repellers(θ, θ̇, prob; K=5) 
    Js = []
    vs = []
    ws = []  
    task = :dodge_qp
    ψ = eval(Symbol(task, :_task_map))
    x = ψ(θ, zero(θ), prob)
    o = get_closest_point(x, prob)  
    # @show o
    e = (o - x)
    S = prob.S[:dodge]
    J = build_jacobian(task, θ, θ̇, S, prob)
    if norm(e) < prob.task_data[:obstacle][:max_range] && o[1]>-0.3
        push!(vs, -K*e)
        push!(Js, J)
        push!(ws, prob.W[:dodge])
    end 
    return Js, vs, ws
end

function qp_compute(θ, θ̇, qmotors, prob)
    model = prob.task_data[:qp][:model]
    # tasks = prob.tasks 
    # qlims = prob.qlims 

    Δt = prob.digit.Δt
    q̇ = model.obj_dict[:q̇]
    Js, vs, ws = build_attractors(θ, θ̇, prob) 
    if :dodge in prob.ψ[:level1]
        Jds, vds, wds = build_repellers(θ, θ̇, prob)
        Js = [Js; Jds]
        vs = [vs; vds]
        ws = [ws; wds]
    end
    # q̇_min, q̇_max = compute_velocity_limits(θ, qlims, Δt)   

    @objective(model, Min, 
            sum([w*(J*q̇ - v)'*(J*q̇ - v) for (J, v, w) in zip(Js, vs, ws)]))

    # model.ext[:q_lims]   = @constraint(model, q̇_min .≤ q̇ .≤ q̇_max)
    # if :com_limit in keys(prob.task_dict) 
    #     Jcom, ẏ_min, ẏ_max, w = compute_com_limits(θ, prob)    
    #     try delete(model, model.ext[:com_lims])  catch nothing; end 
    #     model.ext[:com_lims] = @constraint(model, ẏ_min .≤ (Jcom*q̇)[1] .≤ ẏ_max)
    # end
    optimize!(model)
    Δq = value.(q̇)
    q̇sol = Δq/Δt

    θd = θ+Δq*prob.Δt
    θ̇d = q̇sol

    q_out = copy(θ)
    qdot_out = copy(θ̇ )

    q_out[prob.digit.leg_joint_indices] = θd[prob.digit.leg_joint_indices]
    q_out[prob.digit.arm_joint_indices] = θd[prob.digit.arm_joint_indices] 

    qdot_out[prob.digit.arm_joint_indices] = θ̇d[prob.digit.arm_joint_indices]
    qdot_out[prob.digit.leg_joint_indices] = θ̇d[prob.digit.leg_joint_indices] 

    q_out = clamp.(q_out, prob.digit.θ_min, prob.digit.θ_max)  
    qdot_out = clamp.(qdot_out, prob.digit.θ̇_min, prob.digit.θ̇_max) 

    τ = zero(q_out) 
    return  q_out, qdot_out, τ
end