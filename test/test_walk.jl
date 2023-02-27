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
digit = load_digit(;visualize=visualize)

## task goals
xᵨs = Dict() 
# level 2
xᵨs[:walk] = [0.4, 0.0, 0.0]
# level 1
xᵨs[:left_swing] = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0]
xᵨs[:right_swing] = [0.0, -0.3, 0.0, 0.0, 0.0, 0.0]
xᵨs[:walk_attractor] = [0.0, -0.3, 0.0, 0.0, 0.0, 0.0]

## task maps
ψs = Dict()
ψs[:level4] = [] 
ψs[:level3] = []
ψs[:level2] = [:walk]
ψs[:level1] = [:walk_attractor]

## Task weights
Ws = Dict() 
Ws[:left_swing] = 1e0
Ws[:right_swing] = 1e0
Ws[:walk_attractor] = 1e0

## Priorities
Pr = Dict()
Pr[:left_swing] = 1
Pr[:right_swing] = 1
Pr[:walk_attractor] = 1

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

Ss = Dict() 
Ss[:left_swing] = S_left_swing  
Ss[:right_swing] = S_right_swing

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
                    :vel_des_target=>[0.1,0.0, 0.0],
                    :target_heading=>0.0,
                    :Rz_st=>RotZ(0.0),
                    :indices=>Indices(),
                    :inited=>false,
                    :step_width=>0.26,
                    :digit=>digit 
                    )

data[:obstacle] = Dict(
                    :radius=>0.12,
                    :position=>zeros(3),
                    :max_range=>15.0
        )

data[:diagnostics] = Dict(
                    :qs=>[],
                    :ts=>[]
)

Js = nothing
Obstacles = nothing

problem = FabricProblem(ψs, Js, g, M, Ss, xᵨs, Ws, Obstacles, Pr, data,
zeros(N), zeros(N), 1.0/F, N, digit, 0.0, WalkingMode())

digit.problem = problem
digit.obstacle_force = -0.0
# step(digit) 

#Horizon
T = 20 # seconds
Horizon = T/digit.Δt # timesteps
stand = true
step(digit)

for i = 1:Horizon
    global stand
    if i > 5000 stand = false end
    fabric_controller!(digit;prioritize=prioritize, stand=stand)
    step(digit)
    render_sim(digit, visualize)  
end

if visualize digit.viewer.close() end

# using Plots
# ts = problem.task_data[:diagnostics][:ts]#[1:8000]
# qs = problem.task_data[:diagnostics][:qs]#[1:8000]
# Plots.plot(ts, hcat(qs...)', label=["LeftHipRoll" "LeftHipPitch" "LeftKnee" "RightHipRoll" "RightHipPitch" "RightKnee"])