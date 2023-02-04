mutable struct FabricProblem  
    ψ::Dict
    J::Union{Nothing, Dict{Symbol, Function}}
    g::Function 
    M::Function
    S::Union{Dict{Symbol, Any}, Nothing}
    xᵨ::Union{Nothing, Dict}
    W::Union{Nothing, Dict{Symbol, Float64}}
    Obstacle::Union{Nothing, Dict}
    priorities::Union{Nothing, Dict}
    task_data::Union{Dict, Nothing}
    θ 
    θ̇ 
    Δt
    N
    digit
    t
end

struct CubicBezier 
    p₀::AbstractArray
    τ₀::AbstractArray
    pₙ::AbstractArray
    τₙ::AbstractArray
end

di = DigitInterface
mutable struct Digit
    θ_min
    θ_max
    θ̇_min
    θ̇_max 
    arm_joint_indices
    leg_joint_indices
    joint_names
    motor_names  
    model
    data
    viewer
    obstacle_force
    Δt 
    problem
    function Digit()
        Δt = 5e-4
        θ_min = [-1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -1.0472, -0.698132, -1.0472, -1.2392, -0.35, -0.8779, -0.785398163397, -0.6109, -1.309, -2.5307, -1.7453, -1.3526, -1.0472, -0.698132, -1.57079632679, -0.8727, -0.35, -1.2497, -0.785398163397, -0.6109, -1.309, -2.5307, -1.7453, -1.3526]
        θ_max = [1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308,  1.0472, 0.698132, 1.57079632679, 0.8727, 0.35, 1.2497, 0.785398163397, 0.6109, 1.309, 2.5307, 1.7453, 1.3526, 1.0472, 0.698132, 1.0472, 1.2392, 0.35, 0.8779, 0.785398163397, 0.6109, 1.309, 2.5307, 1.7453, 1.3526] 
	    θ̇_min = [-1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -4.58149, -7.33038, -8.50848, -8.50848, -100, -100, -11.5192, -11.5192, -4.58149, -4.58149, -7.33038, -4.58149, -4.58149, -7.33038, -8.50848, -8.50848, -100,-100, -11.5192, -11.5192, -4.58149, -4.58149, -7.33038, -4.58149]
        θ̇_max = [1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308, 4.58149, 7.33038, 8.50848,8.50848, 100, 100, 11.5192, 11.5192, 4.58149, 4.58149, 7.33038, 4.58149, 4.58149, 7.33038, 8.50848, 8.50848, 100, 100, 11.5192, 11.5192, 4.58149, 4.58149, 7.33038, 4.58149]
        arm_joint_indices = [di.qleftShoulderRoll, di.qleftShoulderPitch, di.qleftShoulderYaw, di.qleftElbow, di.qrightShoulderRoll, di.qrightShoulderPitch, di.qrightShoulderYaw, di.qrightElbow]
        leg_joint_indices = [di.qleftHipRoll, di.qleftHipPitch, di.qleftKnee, di.qrightHipRoll, di.qrightHipPitch, di.qrightKnee]
        obstacle_force = 0.0
        joint_names = ["left-hip-roll", "left-hip-yaw", "left-hip-pitch", 
            "left-knee", "left-shin", "left-tarsus", "left-toe-pitch", 
            "left-toe-roll", "left-shoulder-roll", "left-shoulder-pitch", 
            "left-shoulder-yaw", "left-elbow", "right-hip-roll", "right-hip-yaw", 
            "right-hip-pitch", "right-knee", "right-shin", "right-tarsus", 
            "right-toe-pitch", "right-toe-roll", "right-shoulder-roll", 
            "right-shoulder-pitch", "right-shoulder-yaw", "right-elbow"]
        motor_names = ["left-hip-roll", "left-hip-yaw", "left-hip-pitch", 
            "left-knee", "left-toe-A", "left-toe-B",  "right-hip-roll", "right-hip-yaw", 
            "right-hip-pitch", "right-knee", "right-toe-A", "right-toe-B", "left-shoulder-roll", "left-shoulder-pitch", 
            "left-shoulder-yaw", "left-elbow", "right-shoulder-roll", 
            "right-shoulder-pitch", "right-shoulder-yaw", "right-elbow"]
        new(θ_min, θ_max, θ̇_min, θ̇_max, arm_joint_indices, leg_joint_indices,
             joint_names, motor_names, nothing, nothing, nothing, obstacle_force, Δt, nothing)
    end
end

mutable struct Indices 
    idx_q_st_hiproll_ 
    idx_q_st_hipyaw_
    idx_q_st_hippitch_
    idx_q_st_knee_ 
    idx_q_st_KneeToShin_ 
    idx_q_st_ShinToTarsus_ 

    idx_q_sw_hiproll_ 
    idx_q_sw_hipyaw_ 
    idx_q_sw_hippitch_ 
    idx_q_sw_knee_ 
    idx_q_sw_KneeToShin_ 
    idx_q_sw_ShinToTarsus_ 
 
    idx_m_st_hiproll_ 
    idx_m_st_hipyaw_
    idx_m_st_hippitch_ 
    idx_m_st_knee_ 

    idx_m_sw_hiproll_ 
    idx_m_sw_hipyaw_ 
    idx_m_sw_hippitch_ 
    idx_m_sw_knee_ 
    function Indices()
        new(di.qleftHipRoll
        ,di.qleftHipYaw
        ,di.qleftHipPitch
        ,di.qleftKnee
        ,di.qleftKneeToShin
        ,di.qleftShinToTarsus
        ,di.qrightHipRoll
        ,di.qrightHipYaw
        ,di.qrightHipPitch
        ,di.qrightKnee
        ,di.qrightKneeToShin
        ,di.qrightShinToTarsus
        ,di.qleftHipRoll
        ,di.qleftHipYaw
        ,di.qleftHipPitch
        ,di.qleftKnee
        ,di.qrightHipRoll
        ,di.qrightHipYaw
        ,di.qrightHipPitch
        ,di.qrightKnee)
    end
end 
