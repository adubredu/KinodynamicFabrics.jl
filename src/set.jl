function apply_motor_torques!(τ::Vector{Float64}, digit::Digit)
    for (i, name) in enumerate(digit.motor_names)
        digit.data.actuator(name).ctrl[0] = τ[i]
    end
end

function set_obstacle_position!(pos::Vector{Float64}, digit::Digit)
    digit.data.joint("obstacle").qpos[0:2] = pos 
end

function apply_obstacle_force!(digit::Digit)
    # digit.data.actuator("obstacle").ctrl[0] = digit.obstacle_force
    digit.data.body("obstacle").xfrc_applied[0] = digit.obstacle_force
end
 
function set_right_wrist_orientation(pos, digit::Digit)
    digit.data.joint("right_arm_wrist_roll").qpos[0] = pos[1]
    digit.data.joint("right_arm_wrist_pitch").qpos[0] = pos[2]
    digit.data.joint("right_arm_wrist_yaw").qpos[0] = pos[3]
end

function set_left_wrist_orientation(pos, digit::Digit)
    digit.data.joint("left_arm_wrist_roll").qpos[0] = pos[1]
    digit.data.joint("left_arm_wrist_pitch").qpos[0] = pos[2]
    digit.data.joint("left_arm_wrist_yaw").qpos[0] = pos[3]
end

function left_wrist_grasp(digit::Digit)
    digit.data.actuator("left_grasp").ctrl[0] = 1.0
end

function left_wrist_release(digit::Digit)
    digit.data.joint("left_thumb").qpos[0] = -0.36
    digit.data.joint("left_finger").qpos[0] = -0.36
end

function right_wrist_grasp(digit::Digit) 
    digit.data.actuator("right_grasp").ctrl[0] = 1.0
end

function right_wrist_release(digit::Digit)
    digit.data.joint("right_thumb").qpos[0] = -0.36
    digit.data.joint("right_finger").qpos[0] = -0.36
end