using Revise 
using KinodynamicFabrics 
using KinodynamicFabrics.LinearAlgebra
using KinodynamicFabrics.Rotations
using FiniteDiff

const kf = KinodynamicFabrics 

F = 1e1
N = 30
 
# init Digit
visualize = true
prioritize = false
environment = :cornhole_env 
digit = load_digit(;visualize=visualize, env=environment)

## task goals
xᵨs = Dict() 

# level 2
xᵨs[:walk] = [0.4, 0.0, 0.0]
# level 1 
xᵨs[:com_target] = [0.0, -0.15, 0.92, -0.0, 0.15, 0.92, 0.0, 0.0]
xᵨs[:upper_body_posture] = [-0.15, 1.1, 0, -0.145, 0.15, -1.1, 0, 0.145]  
xᵨs[:open_arms_posture] = [-0.337, 0.463, -0.253, 0, 0.337, -0.463, 0.253, 0]  
xᵨs[:normal_posture] = [-0.15, 1.1, 0, -0.145, 0.15, -1.1, 0, 0.145] 
xᵨs[:cornhole_load_posture] = [0.0, 0.463, 0.253, -0.5, 0.168, -1.432, 0.0, -1.179]
xᵨs[:cornhole_throw_posture] = [-0.15, 1.1, 0, -0.145, 0.168, 0.632, 0.0, -1.179]
xᵨs[:pick_posture] = [0.0, 0.0, 0.0, 0.0, 0.168, -0.463, 0.0, -1.179]
xᵨs[:upper_body_posture]=xᵨs[:open_arms_posture] 

## task maps
ψs = Dict() 
ψs[:level4] = [:mobile_manipulation]
ψs[:level3] = []
ψs[:level2] = []
ψs[:level1] = [:joint_lower_limit, :joint_upper_limit]
level1_task_maps = [:upper_body_posture, :com_target, :walk_attractor]

## Task weights
Ws = Dict()    
Ws[:upper_body_posture] = 1e0
Ws[:joint_lower_limit] = 1e0
Ws[:joint_upper_limit] = 1e0 
Ws[:com_target] = 1e0

## Priorities
Pr = Dict() 
Pr[:upper_body_posture] = 1
Pr[:joint_lower_limit] = 1
Pr[:joint_upper_limit] = 1 
Pr[:com_target] = 1

## dynamics functions
g = kf.dyn.generalized_gravity
M = kf.dyn.mass_inertia_matrix

## selection matrics 
s_left_swing = zeros(N)
s_left_swing[[kf.qleftHipPitch, kf.qleftHipRoll, kf.qleftKnee, kf.qrightKnee]] .= 1.0
S_left_swing = diagm(s_left_swing)

s_right_swing = zeros(N)
s_right_swing[[kf.qrightHipPitch, kf.qrightHipRoll, kf.qrightKnee, kf.qleftKnee]] .= 1.0
S_right_swing = diagm(s_right_swing)

s_arm = zeros(N)
s_arm[digit.arm_joint_indices] .= 1.0
S_arm = diagm(s_arm)

s_left_arm = zeros(N)
s_left_arm[[kf.qleftShoulderRoll, kf.qleftShoulderPitch, kf.qleftShoulderYaw, kf.qleftElbow]] .= 1.0
S_left_arm = diagm(s_arm)

s_right_arm = zeros(N)
s_right_arm[[kf.qrightShoulderRoll, kf.qrightShoulderPitch, kf.qrightShoulderYaw, kf.qrightElbow]] .= 1.0
S_right_arm = diagm(s_arm)

s_leg = zeros(N)
s_leg[digit.leg_joint_indices] .= 1.0
S_leg = diagm(s_leg)

s_walk = zeros(N)
s_walk[[kf.qleftHipRoll, kf.qleftHipPitch, kf.qleftKnee, kf.qrightKnee]] .= 1.0
S_walk = diagm(s_walk)

Ss = Dict() 
Ss[:com_target] = S_leg    
Ss[:upper_body_posture] = S_arm 
Ss[:joint_lower_limit] = S_arm
Ss[:joint_upper_limit] = S_arm  

data = Dict() 

data[:mobile_manipulation] = Dict(
                    :observables=>Dict(),
                    :standing=>true,
                    :digit=>digit,
                    :task_maps=>level1_task_maps,
                    :plan => [(action_symbol=:stand, com_height=0.95, torso_pitch=0.0, period=1.0, torso_roll=0.0),  
                              (action_symbol=:cornhole, ), 
                              ],
                    :action_index => 1)   

data[:stand] = Dict(
                    :state=>:init,
                    :start_time=>0.0,
                    :period=>1.0,
                    :com_height=>0.95,
                    :torso_pitch=>0.0,
                    :torso_roll=>0.0
)   

data[:cornhole] = Dict(
                    :state=>:init,
                    :start_time=>0.0,
                    :pick_period=>1.0,
                    :clasp_period=>1.0,
                    :load_period=>1.0,
                    :throw_period=>1.0,
                    :descend_period=>2.0,
                    :ascend_period=>2.0,
                    :pick_height=>0.43,
                    :pick_torso_pitch=>0.0,
                    :throw_height=>0.9,
                    :throw_torso_pitch=>0.0,
                    :fling=>false,
                    :throw_torque=>3.8, # hard-coded swing torque. A more elegant approach is to solve for it using projectile motion dynamics
)

data[:diagnostics] = Dict(
                    :q=>[],
                    :qdot=>[],
                    :t=>[],
                    :torques=>[],
                    :qarm=>[],
                    :qdotarm => [],
                    :torquearm => []
)

data[:filter] = Dict(
                    :q_filtered => [],
                    :qdot_filtered => [],
                    :filter_parameter=>0.1,
                    :first_iter_pos=>true,
                    :first_iter_vel=>true
)

Js = nothing
Obstacles = nothing

problem = FabricProblem(ψs, Js, g, M, Ss, xᵨs, Ws, Obstacles, Pr, data,
                        zeros(N), zeros(N), 1.0/F, N, digit, 0.0, StandingMode())

digit.problem = problem 

#Horizon
T = 15 # seconds
Horizon = T/digit.Δt # timesteps 
step(digit)

for i = 1:Horizon 
    fabric_controller!(digit;prioritize=prioritize)
    step(digit)
    render_sim(digit, visualize)  
end

if visualize digit.viewer.close() end
:Done
