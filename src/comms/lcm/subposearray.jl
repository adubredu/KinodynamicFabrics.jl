using LCMCore

lc = LCM()

mutable struct pose_t <: LCMType
    timestamp::Int64 
    x::Float64
    y::Float64
    z::Float64
    qw::Float64
    qx::Float64
    qy::Float64
    qz::Float64
end

mutable struct posearray_t <: LCMType
    timestamp::Int64
    num_poses::Int64 
    poses::Vector{pose_t}
end

@lcmtypesetup(pose_t)
@lcmtypesetup(posearray_t, 
    poses=>(num_poses,)
)

function callback(channel::String, message_data::posearray_t)
    ts = message_data.timestamp
    @show ts
end

subscribe(lc, "tag_poses", callback, posearray_t)

while true 
    # try        
        handle(lc)
    # catch
    #     printstyled("error\n";color=:red)
    # end
end