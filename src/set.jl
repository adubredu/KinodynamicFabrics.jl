function apply_motor_torques!(τ::Vector{Float64}, digit::Digit)
    for (i, name) in enumerate(digit.motor_names)
        digit.data.actuator(name).ctrl[0] = τ[i]
    end
end

function set_obstacle_position!(pos::Vector{Float64}, digit::Digit)
    digit.data.joint("obstacle").qpos[0:2] = pos 
end

function apply_obstacle_force!(digit)
    # digit.data.actuator("obstacle").ctrl[0] = digit.obstacle_force
    digit.data.body("obstacle").xfrc_applied[0] = digit.obstacle_force
end
 