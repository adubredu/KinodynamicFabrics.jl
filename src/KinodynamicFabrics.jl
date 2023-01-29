module KinodynamicFabrics

using StaticArrays
using FiniteDiff 
using Reexport
using Rotations
using LinearAlgebra
using DigitInterface
using GLMakie 
using WebSockets, JSON

include("types.jl")
include("utils.jl")

include("kinematics/kinematics.jl")
include("dynamics/dynamics.jl")
include("kinofabrics.jl")
include("lowlevel.jl")

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

export DigitBody,
        Indices

# fabrics
export mm_fabric_compute 

# lowlevel
export mm_fabric_controller,
        stand_control

# kinematics submodule
@reexport using .kinematics
const kin = kinematics
export kin

# dynamics submodule
@reexport using .dynamics
const dyn = dynamics
export dyn

end
