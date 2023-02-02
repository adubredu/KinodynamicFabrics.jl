using Revise 
using KinodynamicFabrics
using KinodynamicFabrics.DigitInterface
using KinodynamicFabrics.LinearAlgebra
using WebSockets, FiniteDiff
using JSON, Rotations, GLMakie

const kfb = KinodynamicFabrics
const di = DigitInterface 

F = 1e1
N = 30

ip = sim_ip
host=:sim 

# ip = robot_ip
# host=:real

# digit
digit = DigitBody()

## task goals
xᵨs = Dict() 
# level 2
xᵨs[:walk] = [0.4, 0.0, 0.0]

# level 1
xᵨs[:left_swing] = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0]
xᵨs[:upper_body_posture] = [-0.15, 1.1, 0, -0.145, 0.15, -1.1, 0, 0.145]
xᵨs[:walk_attractor] = [0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
xᵨs[:com_target] = [0.0, -0.135, 0.92, -0.0, 0.135, 0.92, 0.0, 0.0]
xᵨs[:open_arms_posture] = [-0.337, 0.463, -0.253, 0, 0.337, -0.463, 0.253, 0]
xᵨs[:close_arms_posture] = [0.0, 0.463, 0.253, 0, -0.0, -0.463, -0.253, 0]
xᵨs[:clutch_arms_posture] = [0.0, 0.463, 0.253, -0.5, 0.0, -0.463, -0.253, 0.5]
xᵨs[:normal_posture] = [0.0, 0.463, 0.253, 0, -0.0, -0.463, -0.253, 0]

# xᵨs[:upper_body_posture]=xᵨs[:close_arms_posture] 

## task maps
ψs = Dict() 
ψs[:level4] = [:mobile_manipulation]
ψs[:level3] = []#[:navigate, :bimanual_pickup, :precise_move]
ψs[:level2] = []#[:walk]
ψs[:level1] = [
                # :upper_body_posture,
                # :com_target
                # :walk_attractor
               ]
level1_task_maps = [:upper_body_posture, :com_target, :walk_attractor]

## Task weights
Ws = Dict() 
Ws[:left_elbow_target] = 1e0
Ws[:right_elbow_target] = 1e0
Ws[:left_wrist_target] = 1e0
Ws[:right_wrist_target] = 1e0
Ws[:walk_attractor] = 1e0
Ws[:upper_body_posture] = 1e0
Ws[:joint_lower_limit] = 1e0
Ws[:joint_upper_limit] = 1e0 
Ws[:com_target] = 1e0

## Priorities
Pr = Dict()
Pr[:left_elbow_target] = 1
Pr[:right_elbow_target] = 1
Pr[:left_wrist_target] = 1
Pr[:right_wrist_target] = 1
Pr[:walk_attractor] = 1
Pr[:upper_body_posture] = 1
Pr[:joint_lower_limit] = 1
Pr[:joint_upper_limit] = 1 
Pr[:com_target] = 1

## dynamics functions
g = kfb.dyn.generalized_gravity
M = kfb.dyn.mass_inertia_matrix

## selection matrics 
s_left_swing = zeros(N)
s_left_swing[[di.qleftHipPitch, di.qleftHipRoll, di.qleftKnee, di.qrightKnee]] .= 1.0
S_left_swing = diagm(s_left_swing)

s_right_swing = zeros(N)
s_right_swing[[di.qrightHipPitch, di.qrightHipRoll, di.qrightKnee, di.qleftKnee]] .= 1.0
S_right_swing = diagm(s_right_swing)

s_arm = zeros(N)
s_arm[digit.arm_joint_indices] .= 1.0
S_arm = diagm(s_arm)

s_leg = zeros(N)
s_leg[digit.leg_joint_indices] .= 1.0
S_leg = diagm(s_leg)

s_walk = zeros(N)
s_walk[[di.qleftHipRoll, di.qleftHipPitch, di.qleftKnee, di.qrightKnee]] .= 1.0
S_walk = diagm(s_walk)

Ss = Dict() 
Ss[:com_target] = S_leg  
Ss[:left_swing] = S_left_swing  
Ss[:right_swing] = S_right_swing
Ss[:left_wrist_target] = S_arm 
Ss[:right_wrist_target] = S_arm 
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
                    :vel_des_target=>[0.0,0.0, 0.1],
                    :target_heading=>0.0,
                    :Rz_st=>RotZ(0.0),
                    :indices=>Indices(),
                    :inited=>false,
                    :step_width=>0.27,
                    :digit=>digit 
                    )


data[:mm] = Dict(
                    :observables=>Dict(),
                    :standing=>true,
                    :digit=>digit,
                    :task_maps=>level1_task_maps,
                    :plan => [
                                (action_symbol=:stand, com_height=0.95, torso_pitch=0.0, period=1.0, torso_roll=0.0), 
                              (action_symbol=:precise_move, direction=:side, distance=0.76),
                              (action_symbol=:bimanual_pickup, com_height=0.9, torso_pitch=0.2),
                              (action_symbol=:precise_move, direction=:side, distance=-1.54), 
                              (action_symbol=:precise_move, direction=:forward, distance=1.0),
                              (action_symbol=:bimanual_place, com_height=0.9, torso_pitch=0.2),
                                # (action_symbol=:navigate, waypoint=[0.5, 0.0, 0.0]),
                            #   (action_symbol=:stand, com_height=0.95, torso_pitch=0.0, period=50.0, torso_roll=0.0),
                            #   (action_symbol=:bimanual_pickup, com_height=0.93, torso_pitch=0.4),
                            #   (action_symbol=:navigate, waypoint=[1.5,1.5, 0.875]),
                            #   (action_symbol=:bimanual_place, com_height=0.6, torso_pitch=0.4),
                            
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
                    :tolerance=>0.25,
                    :init_start_time=>false,
                    :init_position=>[0.0,0.0,0.0])

data[:walk_in_place] = Dict(
                    :state => :walk,
                    :period=>5.0,
                    :init_start_time=>false,)

data[:precise_move] = Dict(
                    :direction=>:forward,
                    :distance=>0.0,
                    :Kp=>0.1,
                    :Kd=>0.001,
                    :state=>:init,
                    :init_start_time=>false,
                    :init_position=>[0.0,0.0,0.0],
                    :prev_position=>[0.0,0.0,0.0],
                    :dt=>1e-3,
                    :lim=>0.15,
                    :tolerance=>0.1,
                    :w_start_time=>0.0,
                    :w_period=>3.0,
                    :stand_start_time=>0.0,
                    :stand_period=>2.0,
)

data[:bimanual_pickup] = Dict(
                    :state=>:descend_init, 
                    :flight_time=>3.0,
                    :final_pitch=>0.4,
                    :final_com=>0.6, 
                    :clasp_period=>2.0,
                    :start_time=>0.0,
                    :action_start_time=>0.0,
                    :start_buffer_time=>5.0,
                    :init_start_time=>false)


data[:bimanual_place] = Dict(
                    :state=>:descend_init, 
                    :flight_time=>3.0,
                    :final_pitch=>0.4,
                    :final_com=>0.6, 
                    :open_period=>2.0,
                    :start_time=>0.0,
                    :action_start_time=>0.0,
                    :start_buffer_time=>5.0,
                    :init_start_time=>false)

data[:stand] = Dict(
                    :state=>:init,
                    :start_time=>0.0,
                    :period=>1.0,
                    :com_height=>0.95,
                    :torso_pitch=>0.0,
                    :torso_roll=>0.0
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
zeros(N), zeros(N), 1.0/F, N, digit, 0.0)


publisher_address = ip
llapi_init(publisher_address)    
observation = llapi_observation_t()
command = llapi_command_t()  
observation = llapi_observation_t() 
connect_to_robot(observation, command) 

qi, qidot, qimotors = get_generalized_coordinates(observation) 
problem.θ = qi
problem.θ̇ = qidot   
t_start = observation.time 

mm_params = problem.task_data[:mm]
problem.task_data[:navigate][:init_position] = zeros(3) #qi[[di.qbase_pos_x, di.qbase_pos_y, di.qbase_yaw]]
behavior_switcher(problem; host=host)

while true  
    # @time begin
    if mm_params[:observables][:standing_mode][]
        stand_control(observation, mm_params)
    else
        q, qdot, qmotors = get_generalized_coordinates(observation)
        problem.t = observation.time - t_start 
        # println("running")
        mm_fabric_controller(q, qdot, qmotors, observation, digit, problem) 
        # @show problem.t
    end
    

    sleep(1e-6)
# end
end 
