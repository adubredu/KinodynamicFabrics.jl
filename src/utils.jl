function jvp(f, x, u)
    return FiniteDiff.finite_difference_derivative(t->f(x + t*u), 0.0)
end

function integrate(qi, qidot, qddot, problem)
    α=1e-10; β=1.0; γ=1e-8
    qdot = α*qidot + β*qddot +  γ*qddot*problem.Δt
    q = qi + qdot * problem.Δt
    return q, qdot
end

function get_obstacle_keypoints(c, r; N = 16)
    Δθ = 2π/N
    θs = 0: Δθ : 2π
    keypoints = [[c[1]+r*cos(θ), c[2]+r*sin(θ)] for θ in θs]
    return keypoints
end

function compute_prioritized_jacobian(ψ, t, θ, θ̇ , prob; prioritize=false)
    S = prob.S[t]  
    J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇ , prob), θ) 
    N = I
    if prioritize && !(t == :zmp_upper_limit || t == :zmp_lower_limit) 
        N = compute_nullspace_fast(θ, θ̇ ,prob)
    end
    J = J*S*N
    return J
end

function zmp_limit_task_map(θ, θ̇ , prob::FabricProblem) 
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
    return [params[:upper_limit]-pz[1], pz[1] - params[:lower_limit]]
end

# accurate but slow
function compute_nullspace(θ, θ̇ , prob)
    A = prob.M(θ)
    ψ = zmp_limit_task_map 
    J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇ , prob), θ)
    S = prob.S[:zmp_upper_limit]
    J = J*S
    J_bar = inv(A)*J'*inv(J*inv(A)*J')
    N = I - J_bar * J 
    return N
end

# approximate but faster
function compute_nullspace_fast(θ, θ̇ , prob)
    ψ = zmp_limit_task_map 
    J = FiniteDiff.finite_difference_jacobian(σ->ψ(σ, θ̇ , prob), θ) 
    S = prob.S[:zmp_upper_limit]
    J = J*S  
    J_bar = J'
    N = I - J_bar * J 
    return N
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

# ref: https://math.stackexchange.com/questions/831109/closest-point-on-a-sphere-to-another-point
function get_closest_point(x, prob)
    c = prob.task_data[:obstacle][:position]
    r = prob.task_data[:obstacle][:radius]
    cp = c + ((r/norm(x-c))*(x-c))
    return cp
end

function get_closest_dist_to_obstacle(digit::Digit)
    q, _, _ = get_generalized_coordinates(digit)
    com =  kin.p_base_wrt_feet(q) 
    com[[3, 6]] .+= 0.4  
    pose = [sum(com[[1,4]])/2, sum(com[[2, 5]])/2, sum(com[[3,6]])/2]
    R = RotZYX([q[di.qbase_yaw], q[di.qbase_pitch], q[di.qbase_roll]]...)
    head_pose = R*pose 
    obs_pose = digit.problem.task_data[:obstacle][:position]
    dist=1e10
    if -0.2 <= obs_pose[1] <= 0.4
        ob = [obs_pose[1], obs_pose[2], obs_pose[3]-0.12]
        dist = norm(head_pose-ob) 
    end
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

function unit_smooth(normalized_time::Float64)
    return 1 - cos(normalized_time*2*π) 
end

function azimuth_fxn(digit, time::Float64, duration::Float64, total_rotation::Float64)
    return 100 + unit_smooth(time/duration) * total_rotation
end

activate_fabric!(name::Symbol, problem::FabricProblem, level::Int) = if !(name in problem.ψ[Symbol(:level,level)]) push!(problem.ψ[Symbol(:level,level)], name) end
delete_fabric!(name::Symbol, problem::FabricProblem, level::Int) = deleteat!(problem.ψ[Symbol(:level,level)], findall(x->x==name, problem.ψ[Symbol(:level,level)]))


