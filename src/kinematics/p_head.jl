function p_head(θ)
    head_pose = θ[[qbase_pos_x, qbase_pos_y, qbase_pos_z]]
    head_pose[3] += 0.5
    R = RotZYX(θ[[qbase_yaw, qbase_pitch, qbase_roll]]...)
    pose = R * head_pose
    return pose
end