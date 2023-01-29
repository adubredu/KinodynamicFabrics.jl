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

mutable struct PotentialProblem
    θ
    θ̇  
    tasks::Vector{Symbol}
    W::Dict
    sys
    xᵨ::Dict
    S::Dict
    env
    Δt::Float64
end

struct CubicBezier 
    p₀::AbstractArray
    τ₀::AbstractArray
    pₙ::AbstractArray
    τₙ::AbstractArray
end

di = DigitInterface
mutable struct DigitBody
    θ_min
    θ_max
    gripper
    arm_joint_indices
    leg_joint_indices
    motor_indices
    damping
    function DigitBody()
        θ_min = [-1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -1.797e+308, -1.0472, -0.698132, -1.0472, -1.2392, -0.35, -0.8779, -0.785398163397, -0.6109, -1.309, -2.5307, -1.7453, -1.3526, -1.0472, -0.698132, -1.57079632679, -0.8727, -0.35, -1.2497, -0.785398163397, -0.6109, -1.309, -2.5307, -1.7453, -1.3526]
        θ_max = [1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308, 1.797e+308,  1.0472, 0.698132, 1.57079632679, 0.8727, 0.35, 1.2497, 0.785398163397, 0.6109, 1.309, 2.5307, 1.7453, 1.3526, 1.0472, 0.698132, 1.0472, 1.2392, 0.35, 0.8779, 0.785398163397, 0.6109, 1.309, 2.5307, 1.7453, 1.3526] 
	    damping = [66.849046, 26.112909, 38.05002, 38.05002, 28.553161, 28.553161, 66.849046, 26.112909, 38.05002, 38.05002, 28.553161, 28.553161, 66.849046, 66.849046, 26.112909, 66.849046, 66.849046, 66.849046, 26.112909, 66.849046]
        arm_joint_indices = [di.qleftShoulderRoll, di.qleftShoulderPitch, di.qleftShoulderYaw, di.qleftElbow, di.qrightShoulderRoll, di.qrightShoulderPitch, di.qrightShoulderYaw, di.qrightElbow]
        leg_joint_indices = [di.qleftHipRoll, di.qleftHipPitch, di.qleftKnee, di.qrightHipRoll, di.qrightHipPitch, di.qrightKnee]
        motor_indices = [7, 8, 9, 10, 13,14, 19, 20, 21, 22, 25, 26, 15,16,17,18, 27,28,29,30]
        new(θ_min, θ_max, nothing, arm_joint_indices, leg_joint_indices, motor_indices, damping)
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
