function jvp(f, x, u)
    return FiniteDiff.finite_difference_derivative(t->f(x + t*u), 0.0)
end

function get_obstacle_keypoints(c, r; N = 16)
    Δθ = 2π/N
    θs = 0: Δθ : 2π
    keypoints = [[c[1]+r*cos(θ), c[2]+r*sin(θ)] for θ in θs]
    return keypoints
end

function get_closest_point_to_obstacle(o,r,  x) 
    opoints = get_obstacle_keypoints(o, r; N=32) 
    pts = [[x,b] for b in opoints]
    ind = argmin([norm(σ[1]-σ[2]) for σ in pts ])
    return pts[ind][2]
end

function get_closest_point(o::Vector{Float64}, x, prob::FabricProblem)
    r = prob.Obstacle[:obs1][:r]
    cp = o + r*(x-o)/norm(x-o)
    return cp
end

function get_closest_point(x, prob)
    c = prob.task_data[:obstacle][:position]
    r = prob.task_data[:obstacle][:radius]
    cp = c + r/(norm(x-c))*(x-c)
    return cp
end

function get_closest_dist_to_obstacle(digit::Digit)
    q, _, _ = get_generalized_coordinates(digit)
    com =  kin.p_base_wrt_feet(q) 
    com[[3, 6]] .+= 0.3
    pose = [sum(com[[1,4]])/2, sum(com[[2, 5]])/2, sum(com[[3,6]])/2]
    R = RotZYX([q[di.qbase_yaw], q[di.qbase_pitch], q[di.qbase_roll]]...)
    head_pose = R*pose
    obs_closest_point = get_closest_point(head_pose, digit.problem)
    dist = norm(head_pose-obs_closest_point)
    return dist
end

#= 
https://wrfranklin.org/Research/Short_Notes/pnpoly.html
=#
function pnpoly(nvert::Int64, vertx::AbstractArray,
        verty::AbstractArray, testx::Float64, testy::Float64)
    (i, j, c) = 1, nvert, false
    for i=1:nvert 
        if ( ((verty[i]>testy) != (verty[j]>testy)) &&
            (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
              c = !c
        end
        j = i
    end
    return c
end

function com_in_support_polygon(q)
    com = kin.p_COM(q)
    sp = kin.support_polygon(q)
    corners = [sp[1:2], sp[4:5], sp[10:11], sp[7:8], sp[1:2]]
    res = pnpoly(5, [s[1] for s in corners], 
        [s[2] for s in corners], com[1], com[2])
    return res
end

function plot_support_polygon(q)
    com = kin.p_COM(q)
    sp = kin.support_polygon(q)
    corners = [sp[1:2], sp[4:5], sp[10:11], sp[7:8], sp[1:2]]
    plot([s[1] for s in corners], [s[2] for s in corners], color=:blue)
    plot!([com[1]], [com[2]], seriestype=:scatter, color=:red)
end

function visualize_obstacles!(prob)
    fig = Figure()
    ax = Axis(fig[1,1], aspect=DataAspect(), limits=(-1.,1.,-1.,1.))
    pos = prob.Obstacle[:ball](prob.t)
    ox = Observable(SVector{2, Float64}(pos[1:2]...))
    scatter!(ax, ox; marker=:circle, markersize=0.5, markerspace=:data)
    display(fig)
    return ox
end

function (curve::CubicBezier)(t::Real)
    p₀=curve.p₀; pₙ=curve.pₙ; τₙ=curve.τₙ; τ₀=curve.τ₀
    p = (2(p₀-pₙ) + τₙ + τ₀)*t^3 +
        (3(pₙ-p₀) - τₙ - 2τ₀)*t^2 +
        τ₀*t + p₀
    return p
end

function visualize_swing_trajectory(curve::CubicBezier; Δt=1e-3)
    ts = 0.0:Δt:1.0
    pts = [curve(t) for t in ts]
    pts = hcat(pts...)'
    scatter(pts[:, 1], pts[:, 2])
end 

wrapMax(x, maxi) = (maxi + x%maxi)%maxi
wrap_angle(x) = x# -π + wrapMax(x + π, 2π)

function wrap_to_pi!(angle::Float64)
    if angle > 0.0
        num_rotations = Int(ceil((angle + π)/2π))
        angle = angle - 2π*num_rotations
    else
        num_rotations = Int(ceil((angle - π)/2π))
        angle = angle - 2π*num_rotations
    end 
end

function wrap_to_pi(angle::Float64)
    # if angle > 0.0
    #     num_rotations = Int(ceil((angle + π)/2π))
    #     angle = angle - 2π*num_rotations
    # else
    #     num_rotations = Int(ceil((angle - π)/2π))
    #     angle = angle - 2π*num_rotations
    # end 
    return angle
end

function ypr_to_quat(q::Vector{Float64})
    ypr = RotZYX(q...)
    qr = QuatRotation(ypr)
    # quat = [qr.q.v3, qr.q.v2, qr.q.v1, qr.q.s]
    quat = [qr.q.v1, qr.q.v2, qr.q.v3, qr.q.s]
    return quat
end

function quat_to_ypr(q::Vector{Float64})
    quat = QuatRotation(q)
    zyx = RotZYX(quat)
    ypr = [zyx.theta1, zyx.theta2, zyx.theta3]
    return ypr
end 

function q_frost_to_pinocchio(q_frost)
    q_pin = zeros(31)
    q_pin[1:3] = q_frost[1:3]
    q_pin[4:7] = ypr_to_quat(q_frost[4:6])  
    q_pin[8:end] = q_frost[7:end] 
    return q_pin
end

function q_pinocchio_to_frost(q_pin)
    q_frost = zeros(30)
    q_frost[1:3] = q_pin[1:3]
    q_frost[4:6] = quat_to_ypr([q_pin[7], q_pin[4],q_pin[5],q_pin[6]])
    q_frost[7:end] = q_pin[8:end]
    return q_frost 
end

function v_frost_to_pinocchio(v_frost)
    return v_frost
end

function v_pinocchio_to_frost(v_pin)
    return v_pin
end

function M_pinocchio_to_frost(M_pin)
    return M_pin
end

function g_pinocchio_to_frost(g_pin)
    return g_pin
end

function convert_to_euler_rates2(quat, ω)
    R = RotMatrix(QuatRotation(quat...))
    t2 = R[1]^2
    t3 = R[2]^2
    t4 = R[6]^2
    t5 = R[9]^2
    t6 = t2 + t3
    t7 = 1.0 / t6
    yaw_dot = -R[2] * t7 * (R[4] * ω[3] - R[7] * ω[2]) + R[1] * t7 * (R[5] * ω[3] - R[8] * ω[2])
    pitch_dot = -1.0 / sqrt(-R[3]^2 + 1.0) * (R[6] * ω[3] - R[9] * ω[2])
    roll_dot = (t4 * ω[1] + t5 * ω[1] - R[3] * R[6] * ω[2] - R[3] * R[9] * ω[3]) / (t4 + t5)

    return [yaw_dot, pitch_dot, roll_dot]
end

function get_higher_priorities(t::Symbol, prob)
    priority = prob.priorities[t]
    priors = []
    for k in keys(prob.priorities) 
        if prob.priorities[k] < priority
            push!(priors, (k, prob.priorities[k]))
        end
    end
    sort!(priors, by=x->x[2])
    return [p[1] for p in priors]
end


function compute_prioritized_jacobian(t::Symbol, θ, θ̇ , prob)
    ψ = eval(Symbol(t, :_task_map))
    Jt = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇ , prob), θ)
    ranked_priors = get_higher_priorities(t, prob)
    if isempty(ranked_priors)  return Jt end
    Nu = I(prob.N)
    task_maps = keys(prob.ψ)
    for task in ranked_priors
        if !(task in task_maps) continue end
        ψp = eval(Symbol(task, :_task_map))
        Jp = FiniteDiff.finite_difference_jacobian(σ->ψp(σ, θ̇ , prob), θ)
        # Nu *= I(length(prob.θ)) - (Jp'*Jp)
        Nu *= I(length(θ)) - (Jp'*pinv(Jp'))
    end
    # Nu = diagm(diag(Nu))
    Jt = Jt*Nu  
    return Jt
end 

activate_fabric!(name::Symbol, problem::FabricProblem, level::Int) = if !(name in problem.ψ[Symbol(:level,level)]) push!(problem.ψ[Symbol(:level,level)], name) end
delete_fabric!(name::Symbol, problem::FabricProblem, level::Int) = deleteat!(problem.ψ[Symbol(:level,level)], findall(x->x==name, problem.ψ[Symbol(:level,level)]))


