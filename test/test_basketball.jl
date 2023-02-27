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
environment = :basketball_env # dodge_env, dodge_hoop_env, basketball_env, cornhole_env, package_env
digit = load_digit(;visualize=visualize, env=environment)

## task goals
xᵨs = Dict() 
# level 2
xᵨs[:walk] = [0.4, 0.0, 0.0]

# level 1
xᵨs[:left_swing] = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0]
xᵨs[:upper_body_posture] = [-0.15, 1.1, 0, -0.145, 0.15, -1.1, 0, 0.145]
xᵨs[:walk_attractor] = [0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
xᵨs[:com_target] = [0.0, -0.15, 0.92, -0.0, 0.15, 0.92, 0.0, 0.0]
xᵨs[:open_arms_posture] = [-0.337, 0.463, -0.253, 0, 0.337, -0.463, 0.253, 0]
xᵨs[:close_arms_posture] = [0.0, 0.463, 0.4, 0, -0.0, -0.463, -0.4, 0]
xᵨs[:clutch_arms_posture] = [0.0, 0.463, 0.253, -0.5, 0.0, -0.463, -0.253, 0.5]
xᵨs[:normal_posture] = [-0.15, 1.1, 0, -0.145, 0.15, -1.1, 0, 0.145]
xᵨs[:holding_posture] = [-0.337, 0.463, -0.253, 0, 0.337, -0.463, -0.7, 0]
xᵨs[:hoop_load_posture] = [0,0,0,0,0.17, 2.4, 0, 0.92]
xᵨs[:hoop_throw_posture] = [0,0,0,0,0.17, 0.709, 0, -0.487]
xᵨs[:left_hand_target] = [0.4, 0.2, 1.0]
xᵨs[:right_hand_target] = [0.4, -0.2, 1.0]

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
Ws[:left_elbow_target] = 1e0
Ws[:right_elbow_target] = 1e0
Ws[:left_hand_target] = 1e0
Ws[:right_hand_target] = 1e0
Ws[:walk_attractor] = 1e0
Ws[:upper_body_posture] = 1e0
Ws[:joint_lower_limit] = 1e0
Ws[:joint_upper_limit] = 1e0 
Ws[:com_target] = 1e0

## Priorities
Pr = Dict()
Pr[:left_elbow_target] = 1
Pr[:right_elbow_target] = 1
Pr[:left_hand_target] = 1
Pr[:right_hand_target] = 1
Pr[:walk_attractor] = 1
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
Ss[:left_swing] = S_left_swing  
Ss[:right_swing] = S_right_swing
Ss[:left_hand_target] = S_left_arm 
Ss[:right_hand_target] = S_right_arm 
Ss[:left_elbow_target] = S_arm 
Ss[:right_elbow_target] = S_arm 
Ss[:upper_body_posture] = S_arm 
Ss[:joint_lower_limit] = S_arm
Ss[:joint_upper_limit] = S_arm 
Ss[:walk_attractor] = S_walk

data = Dict()
data[:walk] = Dict(:swing_time=>0.36,
                    :swing_foot=>:left, 
                    :s =>0.0,
                    :t =>0.0,
                    :t0=>0.0,
                    :q=>zeros(30),
                    :qdot=>zeros(30), 
                    :qmotors=>zeros(20),
                    :leftHeelSpring=>0.0, 
                    :rightHeelSpring=>0.0,
                    :p_sw_wrt_st_toe_bos=>zeros(3), 
                    :p_com_wrt_st_bos=>zeros(3),
                    :vel_des_target=>[0.0,0.0, 0.0],
                    :target_heading=>0.0,
                    :Rz_st=>RotZ(0.0),
                    :indices=>Indices(),
                    :inited=>false,
                    :step_width=>0.26,
                    :digit=>digit 
                    )


data[:mobile_manipulation] = Dict(
                    :observables=>Dict(),
                    :standing=>true,
                    :digit=>digit,
                    :task_maps=>level1_task_maps,
                    :plan => [(action_symbol=:stand, com_height=0.95, torso_pitch=0.0, period=5.0, torso_roll=0.0), 
                              (action_symbol=:right_hand_pickup, position=[0.45, -0.12, 0.93], wrist_orientation=[0.0, 1.3, 0.0]),
                              (action_symbol=:precise_move, direction=:side, distance=-0.6),
                              (action_symbol=:stand, com_height=0.95, torso_pitch=0.0, period=1.0, torso_roll=0.0),
                              (action_symbol=:precise_move, direction=:forward, distance=2.0),
                              (action_symbol=:hoop, ),


                            #   (action_symbol=:stand, com_height=0.5, torso_pitch=0.0, period=2.0, torso_roll=0.0), 
                            #   (action_symbol=:jump, com_height=0.8, torso_pitch=0.0, period=1.0, torso_roll=0.0), 

                            #   (action_symbol=:precise_move, direction=:forward, distance=3.6), 
                            #   (action_symbol=:bimanual_place, com_height=0.8, torso_pitch=0.0),
                            
                            #   (action_symbol=:bimanual_place, com_height=0.90, torso_pitch=0.4),
                            #   (action_symbol=:navigate, waypoint=[0.0, -0.1, 0.0]),
                            #   (action_symbol=:bimanual_pickup, com_height=0.6, torso_pitch=0.4),
                            #   (action_symbol=:navigate, waypoint=[0.0, -0.65, 0.0]),
                            #   (action_symbol=:bimanual_place, com_height=0.90, torso_pitch=0.4),
                              ],
                    :action_index => 1)

data[:navigate] = Dict(
                    :goal=>[2.0, 0.0, 0.0],
                    :K=>0.2,
                    :state=>:translate,
                    :K_turn=>1.5,
                    :B_turn=>0.2,
                    :stand_start_time=>0.0,
                    :stand_period=>2.0,
                    :tolerance=>0.2,
                    :init_start_time=>false,
                    :init_position=>[0.0,0.0,0.0])

data[:bimanual_pickup] = Dict(
                    :state=>:descend_init, 
                    :flight_time=>3.0,
                    :final_pitch=>0.4,
                    :final_com=>0.6, 
                    :clasp_period=>2.0,
                    :start_time=>0.0,
                    :action_start_time=>0.0,
                    :start_buffer_time=>1.0,
                    :init_start_time=>false)


data[:bimanual_place] = Dict(
                    :state=>:descend_init, 
                    :flight_time=>3.0,
                    :final_pitch=>0.4,
                    :final_com=>0.6, 
                    :open_period=>2.0,
                    :start_time=>0.0,
                    :action_start_time=>0.0,
                    :start_buffer_time=>1.0,
                    :init_start_time=>false)

data[:left_hand_pickup] = Dict(
                    :state=>:init,
                    :start_time=>0.0,
                    :target=>[0.4, 0.2, 1,0],
                    :wrist_orientation=>0.0,
                    :move_period=>1.0,
                    :grasp_period=>1.0,
                    :raise_height=>0.1,
                    :init_start_time=>false
)

data[:right_hand_pickup] = Dict(
                    :state=>:init,
                    :start_time=>0.0,
                    :target=>[0.4, 0.2, 1,0],
                    :wrist_orientation=>0.0,
                    :move_period=>1.0,
                    :grasp_period=>0.25,
                    :raise_height=>0.4,
                    :raise_period=>1.0,
                    :init_start_time=>false
)

data[:stand] = Dict(
                    :state=>:init,
                    :start_time=>0.0,
                    :period=>1.0,
                    :com_height=>0.95,
                    :torso_pitch=>0.0,
                    :torso_roll=>0.0
)

data[:precise_move] = Dict(
                    :direction=>:forward,
                    :distance=>0.0,
                    :Kp=>0.2,
                    :Kd=>0.001,
                    :state=>:init,
                    :init_start_time=>false,
                    :init_position=>[0.0,0.0,0.0],
                    :prev_position=>[0.0,0.0,0.0],
                    :dt=>1e-3,
                    :limx=>0.5,
                    :limy=>0.2,
                    :tolerance=>0.1,
                    :w_start_time=>0.0,
                    :wx_period=>1.0,
                    :wy_period=>1.0,
                    :stand_start_time=>0.0,
                    :stand_period=>1.0,
)

data[:jump] = Dict(
                    :state=>:init,
                    :start_time=>0.0,
                    :period=>1.0,
                    :com_height=>0.95,
                    :torso_pitch=>0.0,
                    :torso_roll=>0.0,
                    :land_period=>0.5
)

data[:hoop] = Dict(
                    :state => :init,
                    :start_time=>0.0,
                    :descend_period=>1.0,
                    :throw_torso_pitch=>-0.3,
                    :throw_height=>0.9,
                    :load_period=>3.0,
                    :throw_period=>0.5,
                    :fling=>false,
                    :throw_torque=>-3.0

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
T = 30 # seconds
Horizon = T/digit.Δt # timesteps 
step(digit)

for i = 1:Horizon 
    fabric_controller!(digit;prioritize=prioritize)
    step(digit)
    render_sim(digit, visualize)  
end

if visualize digit.viewer.close() end