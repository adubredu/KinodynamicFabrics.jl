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
    θ[di.qleftShinToTarsus] = -θ[di.qleftKnee]
    θ[di.qrightShinToTarsus] = -θ[di.qrightKnee]
    ψ = eval(Symbol(task, :_task_map))
    J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇ , prob), θ)
    J = J*S
    return J
end

function get_task_space_coordinate(task, θ, θ̇, prob)
    θ[di.qleftShinToTarsus] = -θ[di.qleftKnee]
    θ[di.qrightShinToTarsus] = -θ[di.qrightKnee]
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

function compute_velocity_limits(θ, prob, Δt::Float64; K=0.5)
    q_min, q_max = prob.digit.θ_min, prob.digit.θ_max
    q̇_min = K*(q_min - θ)
    q̇_max = K*(q_max - θ)
    return q̇_min, q̇_max
end

function compute_zmp(θ, θ̇ , prob::FabricProblem)
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
    return pz
end

function compute_zmp_upper_limit(θ, θ̇, prob)
    maxlim = 0.1;  K=0.5
    pz = compute_zmp(θ, θ̇ , prob) 
    xmax = K*(maxlim - pz[1])
    J = FiniteDiff.finite_difference_jacobian(σ->compute_zmp(σ, θ̇ , prob), θ)
    w = prob.W[:zmp_upper]
    return J, xmax, w
end

function compute_zmp_lower_limit(θ, θ̇, prob)
    minlim = -0.1; K=0.5
    pz = compute_zmp(θ, θ̇ , prob)
    xmin = K*(minlim - pz[1]) 
    J = FiniteDiff.finite_difference_jacobian(σ->compute_zmp(σ, θ̇ , prob), θ)
    w = prob.W[:zmp_lower]
    return J, xmin, w
end

function qp_compute(θ, θ̇, qmotors, prob)
    model = prob.task_data[:qp][:model]
    digit = prob.digit

    Δt = prob.digit.Δt
    q̇ = model.obj_dict[:q̇]
    Js, vs, ws = build_attractors(copy(θ), copy(θ̇ ), prob) 
    if :dodge in prob.ψ[:level1]
        Jds, vds, wds = build_repellers(copy(θ), copy(θ̇ ), prob)
        Js = [Js; Jds]
        vs = [vs; vds]
        ws = [ws; wds]
    end
    # q̇_min, q̇_max = compute_velocity_limits(θ, prob, Δt)   

    @objective(model, Min, 
            sum([w*(J*q̇ - v)'*(J*q̇ - v) for (J, v, w) in zip(Js, vs, ws)]))

    # model.ext[:q_lims]   = @constraint(model, q̇_min[digit.arm_joint_indices] .≤ q̇[digit.arm_joint_indices] .≤ q̇_max[digit.arm_joint_indices])
    if :zmp_upper in prob.ψ[:level1] && :zmp_lower in prob.ψ[:level1]
        Jcom_upper,  ẏ_max, _ = compute_zmp_upper_limit(θ, θ̇, prob) 
        Jcom_lower, ẏ_min,  _ = compute_zmp_lower_limit(θ, θ̇, prob)    
        try delete(model, model.ext[:zmp_upper_limit])  catch nothing; end 
        try delete(model, model.ext[:zmp_lower_limit])  catch nothing; end  
        model.ext[:zmp_upper_limit] = @constraint(model, (Jcom_upper*q̇)[1] .≤ ẏ_max)
        model.ext[:zmp_lower_limit] = @constraint(model, ẏ_min .≤ (Jcom_lower*q̇)[1])
    end
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

    toe_indices = [di.qleftToePitch, di.qleftToeRoll, di.qrightToePitch, di.qrightToeRoll]
    q_out[toe_indices] = θd[toe_indices]
    qdot_out[toe_indices] = θ̇d[toe_indices] 

    q_out = clamp.(q_out, prob.digit.θ_min, prob.digit.θ_max)  
    qdot_out = clamp.(qdot_out, prob.digit.θ̇_min, prob.digit.θ̇_max) 

    τ = zero(q_out) 
    return  q_out, qdot_out, τ
end