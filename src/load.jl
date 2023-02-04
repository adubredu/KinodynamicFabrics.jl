function initialize_configuration!(digit::Digit)
    default_configuration = Dict(
        "left-hip-roll" => 0.337,
        "left-hip-yaw" => 0.0,
        "left-hip-pitch" => 0.0,
        "left-knee" => 0.0,
        "left-shin" => 0.0,
        "left-tarsus" => 0.0,
        "left-toe-pitch" => -0.126,
        "left-toe-roll" => 0.0,
        "left-shoulder-roll" => -0.15,
        "left-shoulder-pitch" => 1.1,
        "left-shoulder-yaw" => 0.0,
        "left-elbow" => -0.145,
        "right-hip-roll" => -0.337,
        "right-hip-yaw" => 0.0,
        "right-hip-pitch" => 0.0,
        "right-knee" => 0.0,
        "right-shin" => 0.0,
        "right-tarsus" => 0.0,
        "right-toe-pitch" => 0.126,
        "right-toe-roll" => 0.0,
        "right-shoulder-roll" => 0.15,
        "right-shoulder-pitch" => -1.1,
        "right-shoulder-yaw" => 0.0,
        "right-elbow" => 0.145
    )
    for joint in keys(default_configuration)
        digit.data.joint(joint).qpos[0] = default_configuration[joint]
    end
end

function load_digit()
    xml_path = joinpath(dirname(pathof(KinodynamicFabrics)), "model/scene.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    viewer = mujoco_viewer.MujocoViewer(model, data) 
    digit = Digit()
    digit.model = model
    digit.data = data
    digit.viewer = viewer
    initialize_configuration!(digit)
    mujoco.set_mjcb_control(obstacle_controller!)
    return digit
end
 
import Base: step
function step(digit::Digit)
    mujoco.mj_step(digit.model, digit.data) 
end