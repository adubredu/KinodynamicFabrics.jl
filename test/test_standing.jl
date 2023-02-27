using Revise 
using KinodynamicFabrics 
using KinodynamicFabrics.MuJoCo.PythonCall
using KinodynamicFabrics.LinearAlgebra


const kf = KinodynamicFabrics 

F = 1e1
N = 30

# init Digit
visualize = true
digit = load_digit(;visualize=visualize)

## task goals
xᵨs = Dict()

# level 1 
xᵨs[:upper_body_posture] = [-0.15, 1.1, 0, -0.145, 0.15, -1.1, 0, 0.145] 
xᵨs[:com_target] = [0.0, -0.15, 0.92, -0.0, 0.15, 0.92, 0.0, 0.0]
xᵨs[:open_arms_posture] = [-0.337, 0.463, -0.253, 0, 0.337, -0.463, 0.253, 0]
xᵨs[:close_arms_posture] = [0.0, 0.463, 0.253, 0, -0.0, -0.463, -0.253, 0]
xᵨs[:clutch_arms_posture] = [0.0, 0.463, 0.253, -0.5, 0.0, -0.463, -0.253, 0.5]
xᵨs[:normal_posture] = [0.0, 0.463, 0.253, 0, -0.0, -0.463, -0.253, 0]
xᵨs[:upper_body_posture] = xᵨs[:close_arms_posture]

## task maps
ψs = Dict() 
ψs[:level4] = []
ψs[:level3] = [] 
ψs[:level2] = [] 
ψs[:level1] = [
                :upper_body_posture,
                :com_target 
               ] 


## Task weights
Ws = Dict()
Ws[:upper_body_posture] = 1e0
Ws[:com_target] = 1e0
Ws[:dodge] = 1e0

## Priorities
Pr = Dict()
Pr[:upper_body_posture] = 1
Pr[:com_target] = 1

## dynamics functions
g = kf.dyn.generalized_gravity
M = kf.dyn.mass_inertia_matrix

## selection matrics 
s_leg = zeros(N)
s_leg[digit.leg_joint_indices] .= 1.0
S_leg = diagm(s_leg)

s_arm = zeros(N)
s_arm[digit.arm_joint_indices] .= 1.0
S_arm = diagm(s_arm)

Ss = Dict() 
Ss[:upper_body_posture] = S_arm 
Ss[:com_target] = S_leg  
Ss[:dodge] = S_leg  

data = Dict()
data[:obstacle] = Dict(
                :radius=>0.15,
                :position=>zeros(3),
                :max_range=>0.5
)

Js = nothing
Obstacles = nothing

problem = FabricProblem(ψs, Js, g, M, Ss, xᵨs, Ws, Obstacles, Pr, data,
zeros(N), zeros(N), 1.0/F, N, digit, 0.0)

digit.problem = problem
digit.obstacle_force = -0.0
step(digit)

#Horizon
T = 5 # seconds
Horizon = T/digit.Δt # timesteps

for i = 1:Horizon
    fabric_controller!(digit)
    step(digit)
    render_sim(digit, visualize) 
    # @show i
end

if visualize digit.viewer.close() end