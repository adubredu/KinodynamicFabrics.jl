function compute_standing_command_torques(q, qdot, qdes, qdotdes, qmotors, problem::FabricProblem)
    q_motors = collect(qmotors)
    heading = q[di.qbase_yaw] 
    Rz = RotZ(heading)  

    # Aligned toes
    p_left_toe_world = kin.p_toe_pitch_joint_left(q)
    p_right_toe_world = kin.p_toe_pitch_joint_right(q)
    p_left_toe_aligned = Rz' * p_left_toe_world 
    p_right_toe_aligned = Rz' * p_right_toe_world

    # Aligned com 
    p_com_world = kin.p_COM(q)
    v_com_world = kin.v_COM(q, qdot)
    p_com_aligned =  Rz' * p_com_world
    v_com_aligned =  Rz' * v_com_world  

    # Main Fabric
    q_motors_des = copy(q_motors)

    q_motors_des[di.LeftHipRoll] = qdes[di.qleftHipRoll]
    q_motors_des[di.LeftHipPitch] = qdes[di.qleftHipPitch]
    q_motors_des[di.LeftKnee] = qdes[di.qleftKnee]
    q_motors_des[di.RightHipRoll] = qdes[di.qrightHipRoll]
    q_motors_des[di.RightHipPitch] = qdes[di.qrightHipPitch]
    q_motors_des[di.RightKnee] = qdes[di.qrightKnee]

    q_motors_des[di.LeftHipYaw] = 0
    q_motors_des[di.RightHipYaw] = 0

    q_motors_des[di.LeftShoulderRoll] = qdes[di.qleftShoulderRoll]
    q_motors_des[di.LeftShoulderPitch] = qdes[di.qleftShoulderPitch]
    q_motors_des[di.LeftShoulderYaw] = qdes[di.qleftShoulderYaw]
    q_motors_des[di.LeftElbow] = qdes[di.qleftElbow]
    q_motors_des[di.RightShoulderRoll] = qdes[di.qrightShoulderRoll]
    q_motors_des[di.RightShoulderPitch] = qdes[di.qrightShoulderPitch]
    q_motors_des[di.RightShoulderYaw] = qdes[di.qrightShoulderYaw]
    q_motors_des[di.RightElbow] = qdes[di.qrightElbow]  


    # q_motors_des[di.LeftShoulderRoll] = -0.15
    # q_motors_des[di.LeftShoulderPitch] = 1.1
    # q_motors_des[di.LeftShoulderYaw] = 0
    # q_motors_des[di.LeftElbow] = -0.145
    # q_motors_des[di.RightShoulderRoll] = 0.15
    # q_motors_des[di.RightShoulderPitch] = -1.1
    # q_motors_des[di.RightShoulderYaw] = 0
    # q_motors_des[di.RightElbow] = 0.145

    com_midpoint_error = p_com_aligned - 0.5 * (p_left_toe_aligned + p_right_toe_aligned)
    toe_pitch_error = com_midpoint_error[1] 
    Kerr = 1.3
    q_motors_des[di.LeftToeA] =  q_motors_des[di.LeftToeA]  + Kerr*toe_pitch_error
    q_motors_des[di.LeftToeB] = q_motors_des[di.LeftToeB] - Kerr*toe_pitch_error
    q_motors_des[di.RightToeA] = q_motors_des[di.RightToeA] - Kerr*toe_pitch_error
    q_motors_des[di.RightToeB] = q_motors_des[di.RightToeB] + Kerr*toe_pitch_error 

    τ = zeros(di.NUM_MOTORS) 
    q_motors_error = q_motors - q_motors_des    
    
    kp_hiproll_stand = 800   
    kp_hipyaw_stand = 500.0
    kp_hippitch_stand = 500.0
    kp_knee_stand = 800.0
    kp_toe_stand = 300.0  
    kp_knee_comp_stand = 2700
    kd_knee_comp_stand = 300

    kp_shoulderroll_stand = 100.0
    kp_shoulderpitch_stand = 100.0
    kp_shoulderyaw_stand = 100.0
    kp_elbow_stand = 100.0

    τ[di.LeftHipRoll] = -kp_hiproll_stand * q_motors_error[di.LeftHipRoll]
    τ[di.LeftHipYaw] = -kp_hipyaw_stand * q_motors_error[di.LeftHipYaw]
    τ[di.LeftHipPitch] = -kp_hippitch_stand * q_motors_error[di.LeftHipPitch]
    τ[di.LeftKnee] = -kp_knee_stand * q_motors_error[di.LeftKnee]
    τ[di.RightHipRoll] = -kp_hiproll_stand * q_motors_error[di.RightHipRoll]
    τ[di.RightHipYaw] = -kp_hipyaw_stand * q_motors_error[di.RightHipYaw]
    τ[di.RightHipPitch] = -kp_hippitch_stand * q_motors_error[di.RightHipPitch]
    τ[di.RightKnee] = -kp_knee_stand * q_motors_error[di.RightKnee]

    τ[di.LeftShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.LeftShoulderRoll]
    τ[di.LeftShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.LeftShoulderPitch]
    τ[di.LeftShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.LeftShoulderYaw]
    τ[di.LeftElbow] = -kp_elbow_stand * q_motors_error[di.LeftElbow]
    τ[di.RightShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.RightShoulderRoll]
    τ[di.RightShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.RightShoulderPitch]
    τ[di.RightShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.RightShoulderYaw]
    τ[di.RightElbow] = -kp_elbow_stand * q_motors_error[di.RightElbow]

    kd_toe_stand = 1.0
    τ[di.LeftToeA] = -kp_toe_stand * q_motors_error[di.LeftToeA] + kd_toe_stand * v_com_aligned[1]
    τ[di.LeftToeB] = -kp_toe_stand * q_motors_error[di.LeftToeB] - kd_toe_stand * v_com_aligned[1]
    τ[di.RightToeA] = -kp_toe_stand * q_motors_error[di.RightToeA] - kd_toe_stand * v_com_aligned[1]
    τ[di.RightToeB] = -kp_toe_stand * q_motors_error[di.RightToeB] + kd_toe_stand * v_com_aligned[1]

    # knee compensation 
    knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    τ[di.LeftKnee] += knee_comp
    τ[di.RightKnee] += knee_comp 

    v = zeros(di.NUM_MOTORS)   
    v[di.LeftHipRoll] = qdotdes[di.qleftHipRoll]
    v[di.LeftHipPitch] = qdotdes[di.qleftHipPitch]
    v[di.LeftKnee] = qdotdes[di.qleftKnee]
    v[di.RightHipRoll] = qdotdes[di.qrightHipRoll]
    v[di.RightHipPitch] = qdotdes[di.qrightHipPitch]
    v[di.RightKnee] = qdotdes[di.qrightKnee] 
    return τ, v
end


function compute_motor_torques(q, qdot, qdes, qdotdes, q_motors)         
    heading = q[di.qbase_yaw] 
    Rz = RotZ(heading)  

    # Aligned toes
    p_left_toe_world = kin.p_toe_pitch_joint_left(q)
    p_right_toe_world = kin.p_toe_pitch_joint_right(q)
    p_left_toe_aligned = Rz' * p_left_toe_world 
    p_right_toe_aligned = Rz' * p_right_toe_world

    # Aligned com 
    p_com_world = kin.p_COM(q)
    v_com_world = kin.v_COM(q, qdot)
    p_com_aligned =  Rz' * p_com_world
    v_com_aligned =  Rz' * v_com_world  

    # Main Fabric
    q_motors_des = copy(q_motors)

    q_motors_des[di.LeftHipRoll] = qdes[di.qleftHipRoll]
    q_motors_des[di.LeftHipPitch] = qdes[di.qleftHipPitch]
    q_motors_des[di.LeftKnee] = qdes[di.qleftKnee]
    q_motors_des[di.RightHipRoll] = qdes[di.qrightHipRoll]
    q_motors_des[di.RightHipPitch] = qdes[di.qrightHipPitch]
    q_motors_des[di.RightKnee] = qdes[di.qrightKnee]

    q_motors_des[di.LeftHipYaw] = 0
    q_motors_des[di.RightHipYaw] = 0

    q_motors_des[di.LeftShoulderRoll] = qdes[di.qleftShoulderRoll]
    q_motors_des[di.LeftShoulderPitch] = qdes[di.qleftShoulderPitch]
    q_motors_des[di.LeftShoulderYaw] = qdes[di.qleftShoulderYaw]
    q_motors_des[di.LeftElbow] = qdes[di.qleftElbow]
    q_motors_des[di.RightShoulderRoll] = qdes[di.qrightShoulderRoll]
    q_motors_des[di.RightShoulderPitch] = qdes[di.qrightShoulderPitch]
    q_motors_des[di.RightShoulderYaw] = qdes[di.qrightShoulderYaw]
    q_motors_des[di.RightElbow] = qdes[di.qrightElbow]  

    com_midpoint_error = p_com_aligned - 0.5 * (p_left_toe_aligned + p_right_toe_aligned)
    toe_pitch_error = com_midpoint_error[1] 
    Kerr = 1.3
    q_motors_des[di.LeftToeA] =  q_motors_des[di.LeftToeA]  + Kerr*toe_pitch_error
    q_motors_des[di.LeftToeB] = q_motors_des[di.LeftToeB] - Kerr*toe_pitch_error
    q_motors_des[di.RightToeA] = q_motors_des[di.RightToeA] - Kerr*toe_pitch_error
    q_motors_des[di.RightToeB] = q_motors_des[di.RightToeB] + Kerr*toe_pitch_error 

    τ = zeros(di.NUM_MOTORS) 
    q_motors_error = q_motors - q_motors_des    
    
    kp_hiproll_stand = 80   
    kp_hipyaw_stand = 50.0
    kp_hippitch_stand = 50.0
    kp_knee_stand = 80.0
    kp_toe_stand = 3.0  
    kp_knee_comp_stand = 270
    kd_knee_comp_stand = 30

    kp_shoulderroll_stand = 100.0
    kp_shoulderpitch_stand = 100.0
    kp_shoulderyaw_stand = 100.0
    kp_elbow_stand = 100.0

    τ[di.LeftHipRoll] = -kp_hiproll_stand * q_motors_error[di.LeftHipRoll]
    τ[di.LeftHipYaw] = -kp_hipyaw_stand * q_motors_error[di.LeftHipYaw]
    τ[di.LeftHipPitch] = -kp_hippitch_stand * q_motors_error[di.LeftHipPitch]
    τ[di.LeftKnee] = -kp_knee_stand * q_motors_error[di.LeftKnee]
    τ[di.RightHipRoll] = -kp_hiproll_stand * q_motors_error[di.RightHipRoll]
    τ[di.RightHipYaw] = -kp_hipyaw_stand * q_motors_error[di.RightHipYaw]
    τ[di.RightHipPitch] = -kp_hippitch_stand * q_motors_error[di.RightHipPitch]
    τ[di.RightKnee] = -kp_knee_stand * q_motors_error[di.RightKnee]

    τ[di.LeftShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.LeftShoulderRoll]
    τ[di.LeftShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.LeftShoulderPitch]
    τ[di.LeftShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.LeftShoulderYaw]
    τ[di.LeftElbow] = -kp_elbow_stand * q_motors_error[di.LeftElbow]
    τ[di.RightShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[di.RightShoulderRoll]
    τ[di.RightShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[di.RightShoulderPitch]
    τ[di.RightShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[di.RightShoulderYaw]
    τ[di.RightElbow] = -kp_elbow_stand * q_motors_error[di.RightElbow]

    kd_toe_stand = 1.0
    τ[di.LeftToeA] = -kp_toe_stand * q_motors_error[di.LeftToeA] + kd_toe_stand * v_com_aligned[1]
    τ[di.LeftToeB] = -kp_toe_stand * q_motors_error[di.LeftToeB] - kd_toe_stand * v_com_aligned[1]
    τ[di.RightToeA] = -kp_toe_stand * q_motors_error[di.RightToeA] - kd_toe_stand * v_com_aligned[1]
    τ[di.RightToeB] = -kp_toe_stand * q_motors_error[di.RightToeB] + kd_toe_stand * v_com_aligned[1]

    # knee compensation 
    knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    τ[di.LeftKnee] += knee_comp
    τ[di.RightKnee] += knee_comp 

    v = zeros(di.NUM_MOTORS)   
    v[di.LeftHipRoll] = qdotdes[di.qleftHipRoll]
    v[di.LeftHipPitch] = qdotdes[di.qleftHipPitch]
    v[di.LeftKnee] = qdotdes[di.qleftKnee]
    v[di.RightHipRoll] = qdotdes[di.qrightHipRoll]
    v[di.RightHipPitch] = qdotdes[di.qrightHipPitch]
    v[di.RightKnee] = qdotdes[di.qrightKnee]

    return τ
end

function fabric_controller!(digit::Digit)
    q, qdot, qmotors = get_generalized_coordinates(digit) 
    qdes, qdotdes, torqdes = fabric_compute(q, qdot, qmotors, digit.problem)
    τ = compute_motor_torques(q, qdot, qdes, qdotdes, qmotors)         
    apply_motor_torques!(τ, digit)
    apply_obstacle_force!(digit)
end