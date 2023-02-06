module KinodynamicFabrics

using StaticArrays
using FiniteDiff 
using Reexport
using Rotations
using LinearAlgebra
using DigitInterface   
using CSV
using MuJoCo
using MuJoCo.PythonCall

include("types.jl")
include("utils.jl")

include("kinematics/kinematics.jl")
include("dynamics/dynamics.jl")
include("kinofabrics.jl")
include("lowlevel.jl")

include("set.jl")
include("get.jl")
include("load.jl")

export FabricProblem


export jvp,
       get_closest_point,
       display_goal!, 
       activate_fabric!,
       delete_fabric!,
       com_in_support_polygon,
       plot_support_polygon,
       visualize_obstacles!,
       visualize_swing_trajectory,
       wrap_to_pi!,
       wrap_to_pi,
       wrap_angle,
       behavior_switcher

export Digit,
        Indices

# fabrics
export fabric_compute 

export qp_compute

# lowlevel
export  fabric_controller!,
        qp_controller!

# sim
export load_digit,
        step,
        get_generalized_coordinates,
        apply_motor_torques!,
        apply_obstacle_force!, 
        render_sim

# kinematics submodule
@reexport using .kinematics
const kin = kinematics
export kin

# dynamics submodule
@reexport using .dynamics
const dyn = dynamics
export dyn

end
