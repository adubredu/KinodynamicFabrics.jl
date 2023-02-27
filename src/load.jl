function initialize_configuration!(digit::Digit)
    default_configuration = Dict(
        "left-hip-roll" => 0.31,
        "left-hip-yaw" => 0.0,
        "left-hip-pitch" => 0.2,
        "left-knee" => 0.19,
        "left-shin" => 0.0,
        "left-tarsus" => 0.0,
        "left-toe-pitch" => -0.126,
        "left-toe-roll" => 0.0,
        "left-shoulder-roll" => -0.15,
        "left-shoulder-pitch" => 1.1,
        "left-shoulder-yaw" => 0.0,
        "left-elbow" => -0.145,
        "right-hip-roll" => -0.31,
        "right-hip-yaw" => 0.0,
        "right-hip-pitch" => -0.2,
        "right-knee" => -0.19,
        "right-shin" => 0.0,
        "right-tarsus" => 0.0,
        "right-toe-pitch" => 0.126,
        "right-toe-roll" => 0.0,
        "right-shoulder-roll" => 0.0,
        "right-shoulder-pitch" => -0,
        "right-shoulder-yaw" => 0.0,
        "right-elbow" => 0.0
    )
    for joint in keys(default_configuration)
        digit.data.joint(joint).qpos[0] = default_configuration[joint]
    end
end

function get_env_path(env::Symbol)
    if env == :default_env
        path = "model/default_scene.xml"
    elseif env == :dodge_env
        path = "model/dodge_scene.xml"
    elseif env == :dodge_hoop_env
        path = "model/dodge_hoop_scene.xml"
    elseif env == :basketball_env
        path = "model/basketball_scene.xml"
    elseif env == :cornhole_env
        path = "model/cornhole_scene.xml"
    elseif env == :package_env
        path = "model/package_scene.xml"
    else
        path = "model/default_scene.xml"
    end
    return path
end


function load_digit(;visualize=false, save_video=false, env=:default_env)
    scene_path = get_env_path(env)
    xml_path = joinpath(dirname(pathof(KinodynamicFabrics)), scene_path)
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    digit = Digit()
    digit.model = model
    digit.data = data
    digit.model.actuator_gainprm[0, 0] = 1
    initialize_configuration!(digit) 
    if visualize 
        if save_video
            viewer = mujoco_viewer.MujocoViewer(model, data, "offscreen", hide_menus=true)#, 
                                # width=1200, height=900) 
        else
            viewer = mujoco_viewer.MujocoViewer(model, data,  hide_menus=true)
        end
        digit.viewer = viewer
    end
    return digit
end

function update_obstacle_position!(digit::Digit)
    if haskey(digit.problem.task_data, :obstacle)
        p = digit.data.body("obstacle").xpos
        digit.problem.task_data[:obstacle][:position][1] = pyconvert(Float64, p[0])
        digit.problem.task_data[:obstacle][:position][2] = pyconvert(Float64, p[1])
        digit.problem.task_data[:obstacle][:position][3] = pyconvert(Float64, p[2])
    end
end
 
import Base: step
function step(digit::Digit)
    mujoco.mj_step(digit.model, digit.data) 
    update_obstacle_position!(digit)
    digit.problem.t = pyconvert(Float64, digit.data.time)
end

function render_sim(digit, visualize::Bool; fps=50)
    if visualize &&  pyconvert(Bool, digit.viewer.is_alive)
        if round(pyconvert(Float64, digit.data.time % (fps*digit.Δt)); digits=3) == 0.0            
            digit.viewer.render()
        end
    end
end

function render_sim(digit, distance::Float64, duration::Float64, 
                   total_angle::Float64, pixels; fps=50, elevation=-30.0,
                            save_video=false)
    if pyconvert(Bool, digit.viewer.is_alive)
        if round(pyconvert(Float64, digit.data.time % (fps*digit.Δt)); digits=3) == 0.0            
            digit.viewer.cam.distance = distance
            digit.viewer.cam.elevation = elevation
            digit.viewer.cam.lookat = [0.4, 0.5, 0.1]
            digit.viewer.cam.azimuth = azimuth_fxn(digit, digit.problem.t, duration, total_angle)
            if save_video
                img = digit.viewer.read_pixels()
                push!(pixels, img)
            else
                digit.viewer.render()
            end
        end
    end
end