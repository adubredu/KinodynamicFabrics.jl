function initialize_solver(N; Δt=1e-3)
    model = JuMP.Model(OSQP.Optimizer) 
    set_silent(model)
    @variable(model, q̇[1:N])
    return model
end

function build_attractors(θ, θ̇, prob; K=0.5) 
    Js = []
    vs = []
    ws = [] 
    for task in in prob.ψ[:level1] 
        if task != :com_limit && task != :dodge
            S = prob.S[:task]
            push!(Js, prob.J[:task](θ)*S)
            push!(ws, prob.W[:task])
            p_current = task.task_map(θ)
            e = pose_error(task.target, vec(p_current))
            push!(vs, K*e)
        end
    end
    return Js, vs, ws
end

function qp_compute(θ, θ̇, qmotors, prob)
    model = prob.model
    tasks = prob.tasks 
    qlims = prob.qlims 

    Δt = prob.Δt
    q̇ = model.obj_dict[:q̇]
    Js, vs, ws = build_attractors(θ, θ̇, prob) 
    q̇_min, q̇_max = compute_velocity_limits(θ, qlims, Δt)   

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

    θd = θ+Δq*problem.Δt
    θ̇d = q̇sol

    q_out = copy(q)
    qdot_out = copy(qdot)

    q_out[problem.digit.leg_joint_indices] = θd[problem.digit.leg_joint_indices]
    q_out[problem.digit.arm_joint_indices] = θd[problem.digit.arm_joint_indices] 

    qdot_out[problem.digit.arm_joint_indices] = θ̇d[problem.digit.arm_joint_indices]
    qdot_out[problem.digit.leg_joint_indices] = θ̇d[problem.digit.leg_joint_indices] 

    q_out = clamp.(q_out, problem.digit.θ_min, problem.digit.θ_max)  
    qdot_out = clamp.(qdot_out, problem.digit.θ̇_min, problem.digit.θ̇_max) 

    τ = zero(q_out) 
    return  q_out, qdot_out, τ
end