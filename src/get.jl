function get_generalized_coordinates(digit::Digit)  
    data = digit.data
    base_orientation_quat = data.qpos[3:6] 
    base_orientation = quat_to_ypr([pyconvert(Float64, a) for a in base_orientation_quat])
    base_position = data.qpos[0:2]
    base_linear_velocity = data.qvel[0:2]
    base_angular_velocity = data.qvel[3:5]
    q = [data.joint(name).qpos[0] for name in digit.joint_names] 
    q = [ base_position..., base_orientation..., q...]
    q̇ = [data.joint(name).qvel[0] for name in digit.joint_names] 
    q̇ = [base_linear_velocity..., base_angular_velocity..., q̇...]
    q = [pyconvert(Float64, a) for a in q]
    q̇ = [pyconvert(Float64, b) for b in q̇] 
    qmotors = [data.joint(name).qpos[0] for name in digit.motor_names]
    qmotors = [pyconvert(Float64, a) for a in qmotors]
    return q, q̇, qmotors
end