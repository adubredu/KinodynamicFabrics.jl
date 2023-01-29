path = @__DIR__
joint_coeffs = CSV.read(joinpath(path, "ankle_joint_coeffs.csv"), DataFrame)
motor_coeffs = CSV.read(joinpath(path, "ankle_motor_coeffs.csv"), DataFrame)

function qm_toe(qtoePitch::Float64, qtoeRoll::Float64, side, foot)
    joint_name = foot*"_toe_"*side
    c0, c1, c2, c3, c4, c5 = motor_coeffs[!, joint_name] 
    x1 = qtoePitch
    x2 = qtoeRoll
    qmtoe = c0 + c1*x1 + c2*x2 + c3*x1*x2 + c4*x1^2 + c5*x2^2
    return qmtoe
end

function qp_toe(qtoeA::Float64, qtoeB::Float64, side, foot)
    joint_name = foot*"_toe_"*side
    c0, c1, c2, c3, c4, c5 = joint_coeffs[!, joint_name] 
    x1 = qtoeA
    x2 = qtoeB
    qptoe = c0 + c1*x1 + c2*x2 + c3*x1*x2 + c4*x1^2 + c5*x2^2
    return qptoe
end