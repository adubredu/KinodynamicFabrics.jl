# helpers 
function reach_map(x, params::Dict)
    res = x - params[:goal] 
    return res 
end

function reach_force(x, params::Dict)
    K = params[:K] 
    f = -K*x 
    return f 
end

function get_floating_pose(θ, params::Dict)
    p = θ[[di.qbase_pos_x, di.qbase_pos_y, di.qbase_yaw]]-params[:init_position]
    return p
end

function filter_coordinates(vs, prob, val=:vel)
    param = prob.task_data[:filter]
    a = param[:filter_parameter]
    if val == :vel
        if param[:first_iter_vel]
            qdot_filtered = vs
            param[:qdot_filtered] = vs
            param[:first_iter_vel] = false
        else
            qdot_filtered = (1-a)*param[:qdot_filtered] + a*vs
            param[:qdot_filtered] = qdot_filtered
        end
        return qdot_filtered
    else
        if param[:first_iter_pos]
            q_filtered = vs
            param[:q_filtered] = vs
            param[:first_iter_pos] = false
        else
            q_filtered = (1-a)*param[:q_filtered] + a*vs
            param[:q_filtered] = q_filtered
        end
        return q_filtered
    end

end