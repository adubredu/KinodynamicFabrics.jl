function initialize_solver(N; Δt=1e-3)
    model = JuMP.Model(OSQP.Optimizer) 
    set_silent(model)
    @variable(model, q̇[1:N])
    return model
end

function build_attractors(tasks::AbstractArray{KinTask}, θ, Δt; K=0.5) 
    Js = []
    vs = []
    ws = [] 
    for task in tasks 
        if task.name != :com_limit && task.name != :dodge
            S = task.selection_matrix
            push!(Js, task.task_map_jacobian(θ)*S)
            push!(ws, task.weight)
            p_current = task.task_map(θ)
            e = pose_error(task.target, vec(p_current))
            push!(vs, K*e)
        end
    end
    return Js, vs, ws
end

function qp_solve(θ, θ̇, prob)
    model = prob.model
    tasks = prob.tasks 
    qlims = prob.qlims 

    Δt = prob.Δt
    q̇ = model.obj_dict[:q̇]
    Js, vs, ws = build_attractors(tasks, θ, Δt) 
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
    return θ+Δq, q̇sol, zero(θ)
end