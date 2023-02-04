function load_digit(;init_robot_position=[0.0,0.0,0.95],
                     init_obstacle_position=[3.0, 0.0, 1.0])
    xml_path = joinpath(dirname(pathof(KinodynamicFabrics)), "model/scene.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    viewer = mujoco_viewer.MujocoViewer(model, data)
    viewer._run_speed = 2.0
    digit = Digit()
    digit.model = model
    digit.data = data
    digit.viewer = viewer
    return digit
end
 
import Base: step
function step(digit::Digit)
    mujoco.mj_step(digit.model, digit.data)
end