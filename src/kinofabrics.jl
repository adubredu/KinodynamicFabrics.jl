include("helpers.jl")
include("task_maps.jl")

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
    ẍ = -k*norm(ẋ)^2*δx  
    M = W * I(N)
    return (M, ẍ)
end

function upper_body_posture_fabric(x, ẋ, prob::FabricProblem) 
    λᵪ = 0.25; k = 4.0;  β=0.75; α=1e-3
    N = length(x)
    W = prob.W[:upper_body_posture] 
    k = W*k 
    Δx = x - prob.xᵨ[:upper_body_posture]
    ψ(θ) = 0.5*θ'*k*θ
    δx = FiniteDiff.finite_difference_gradient(ψ, Δx)
    ẍ = -(k + α*(norm(ẋ))^2)*δx - β*ẋ 
    M = W * I(N)
    return (M, ẍ) 
end

function lower_body_posture_fabric(x, ẋ, prob::FabricProblem) 
    λᵪ = 0.25; k = 4.0;  β=0.75; α=1e-3
    N = length(x) 
    W = prob.W[:lower_body_posture] 
    k = W*k 
    Δx = x - prob.xᵨ[:lower_body_posture]
    ψ(θ) = 0.5*θ'*k*θ
    δx = FiniteDiff.finite_difference_gradient(ψ, Δx)  
    ẍ = -(k + α*(norm(ẋ))^2)*δx - β*ẋ 
    M = W * I(N) 
    return (M, ẍ) 
end

function com_target_fabric(x, ẋ, prob::FabricProblem)
    k = 5.0; αᵩ = 10.0; β=0.5; λ = 0.25; α=1e-3
    N = length(x)
    W = prob.W[:com_target]
    k = W*k  
    Δx = x - prob.xᵨ[:com_target][1:6]
    ψ(θ) = 0.5*θ'*k*θ
    δx = FiniteDiff.finite_difference_gradient(ψ, Δx) 
    ẍ = -(k + α*(norm(ẋ))^2)*δx - β*ẋ 
    M = W * I(N)
    return (M, ẍ)
end

function left_hand_target_fabric(x, ẋ, prob::FabricProblem)
    k = 5.0; αᵩ = 10.0; β=0.5; λ = 0.25; α=1e-3
    N = length(x)
    W = prob.W[:left_hand_target]
    k = W*k  
    Δx = x - prob.xᵨ[:left_hand_target] 
    ψ(θ) = 0.5*θ'*k*θ
    δx = FiniteDiff.finite_difference_gradient(ψ, Δx) 
    ẍ = -(k + α*(norm(ẋ))^2)*δx - β*ẋ 
    M = W * I(N)
    return (M, ẍ)
end

function right_hand_target_fabric(x, ẋ, prob::FabricProblem)
    k = 5.0; αᵩ = 10.0; β=0.5; λ = 0.25; α=1e-3 
    N = length(x)
    W = prob.W[:right_hand_target]
    k = W*k  
    Δx = x - prob.xᵨ[:right_hand_target] 
    ψ(θ) = 0.5*θ'*k*θ
    δx = FiniteDiff.finite_difference_gradient(ψ, Δx) 
    ẍ = -(k + α*(norm(ẋ))^2)*δx - β*ẋ 
    M = W * I(N)
    return (M, ẍ)
end

function dodge_fabric(x, ẋ, prob::FabricProblem)
    W = prob.W[:dodge]; k=1.0; λ = 0.2; α=1e-3 
    K = W*k
    N = length(x)
    max_range = prob.task_data[:obstacle][:max_range]
    s = [v > max_range ? 0.0 : 1.0 for v in x]
    obs_pose = prob.task_data[:obstacle][:position]
    if obs_pose[1] < -0.2 s = zero(s) end
    ϕ(σ) = (K/2) .* s .* (max_range .- σ)./(max_range .* σ).^2
    δₓ = FiniteDiff.finite_difference_jacobian(ϕ, x)  
    ẍ = -(K + α*(norm(ẋ))^2)*diag(δₓ)
    M = norm(ẍ)*I(N)  
    # M = W*(s[1]*(λ/(x'x)))*I(N)
    return (M, ẍ)
end 

function zmp_upper_limit_fabric(x, ẋ, prob::FabricProblem)
    λ = 0.25; α=1e-15
    s = zero(ẋ)
    K = prob.W[:zmp_upper_limit]
    for i in eachindex(s) s[i] = ẋ[i] < 0.0 ? 1 : 0 end
    M = diagm(s.*(λ./x))
    ψ(θ) = (K/2) .* s .* (1 ./ θ)
    δx = FiniteDiff.finite_difference_jacobian(ψ, x)  
    ẍ = -(K + α*(norm(ẋ))^2)*δx
    ẍ = vec(ẍ) 
    return (M, ẍ)
end

function zmp_lower_limit_fabric(x, ẋ, prob::FabricProblem)
    λ = 0.25; α=1e-15
    s = zero(ẋ)
    K = prob.W[:zmp_lower_limit]
    for i in eachindex(s) s[i] = ẋ[i] < 0.0 ? 1 : 0 end
    M = diagm(s.*(λ./x))
    ψ(θ) = (K/2) .* s .* (1 ./ θ)
    δx = FiniteDiff.finite_difference_jacobian(ψ, x)  
    ẍ = -(K + α*(norm(ẋ))^2)*δx
    ẍ = vec(ẍ) 
    return (M, ẍ)
end

function joint_lower_limit_fabric(x, ẋ, prob::FabricProblem)
    λ = 0.25; α=1e-3
    s = zero(ẋ)
    K = prob.W[:joint_lower_limit]
    for i in eachindex(s) s[i] = ẋ[i] < 0.0 ? 1 : 0 end
    M = diagm(s.*(λ./x))
    ψ(θ) = (K/2) .* s .* (1 ./ θ)
    δx = FiniteDiff.finite_difference_jacobian(ψ, x)  
    ẍ = -(K + α*(norm(ẋ))^2)*δx
    ẍ = diag(ẍ)  
    return (M, ẍ) 
end

function joint_upper_limit_fabric(x, ẋ, prob::FabricProblem)
    λ = 0.25; α=1e-3
    s = zero(ẋ)
    K = prob.W[:joint_upper_limit]
    for i in eachindex(s) s[i] = ẋ[i] < 0.0 ? 1 : 0 end
    M = diagm(s.*(λ./x))
    ψ(θ) = (K/2) .* s .* (1 ./ θ)
    δx = FiniteDiff.finite_difference_jacobian(ψ, x) 
    ẍ = -(K + α*(norm(ẋ))^2)*δx
    ẍ = diag(ẍ) 
    return (M, ẍ)
end


## Solve
function fabric_eval(x, ẋ, name::Symbol, prob::FabricProblem)
    M = nothing; ẍ = nothing 
    fabricname = Symbol(name, :_fabric)
    ϕ = eval(fabricname)
    M, ẍ = ϕ(x, ẋ, prob)
    return (M, ẍ)
end

function fabric_solve(θ, θ̇ , qmotors,  prob::FabricProblem; prioritize=true)
    xₛ = []; ẋₛ = []; cₛ = []; 
    Mₛ = []; ẍₛ = []; Jₛ =  [] 
    for t in prob.ψ[:level4]
        ψ = eval(Symbol(t, :_task_map))
        ψ(θ, θ̇ , qmotors,  prob)
    end
    for t in prob.ψ[:level3]
        ψ = eval(Symbol(t, :_task_map))
        ψ(θ, θ̇ , qmotors,  prob)
    end
    for t in prob.ψ[:level2]
        ψ = eval(Symbol(t, :_task_map))
        ψ(θ, θ̇ , qmotors,  prob)
    end 
    for t in prob.ψ[:level1]
        ψ = eval(Symbol(t, :_task_map)) 
        J = compute_prioritized_jacobian(ψ, t, θ, θ̇ , prob;prioritize=prioritize)
        x = ψ(θ, θ̇ , prob)  
        ẋ = J*θ̇ 
        c = zero(x) 
        M, ẍ = fabric_eval(x, ẋ, t, prob)   
        push!(xₛ, x); push!(ẋₛ, ẋ); push!(cₛ, c)
        push!(Mₛ, M); push!(ẍₛ, ẍ); push!(Jₛ, J) 
    end   
    Mᵣ = sum([J' * M * J for (J, M) in zip(Jₛ, Mₛ)])
    fᵣ = sum([J' * M * (ẍ - c) for (J, M, ẍ, c) in zip(Jₛ, Mₛ, ẍₛ, cₛ)])
    Mᵣ = convert(Matrix{Float64}, Mᵣ)  
    q̈ = pinv(Mᵣ) * fᵣ
    return  q̈  
end

function fabric_compute(q, qdot, qmotors,  problem; prioritize=true)
    θ̈d = fabric_solve(copy(q), copy(qdot), qmotors,  problem; prioritize=prioritize)  
    θd, θ̇d = integrate(q, qdot, θ̈d, problem)

    q_out = copy(q)
    qdot_out = copy(qdot)

    toe_indices = [di.qleftToePitch, di.qleftToeRoll, di.qrightToePitch, di.qrightToeRoll]

    q_out[problem.digit.leg_joint_indices] = θd[problem.digit.leg_joint_indices]
    q_out[problem.digit.arm_joint_indices] = θd[problem.digit.arm_joint_indices]
    q_out[toe_indices] = θd[toe_indices]  

    qdot_out[problem.digit.arm_joint_indices] = θ̇d[problem.digit.arm_joint_indices]
    qdot_out[problem.digit.leg_joint_indices] = θ̇d[problem.digit.leg_joint_indices] 
    qdot_out[toe_indices] = θ̇d[toe_indices] 


    q_out = clamp.(q_out, problem.digit.θ_min, problem.digit.θ_max)  
    qdot_out = clamp.(qdot_out, problem.digit.θ̇_min, problem.digit.θ̇_max) 

    τ = zero(q_out) 
    return  q_out, qdot_out, τ
end