module SubPose 

using CxxWrap

@wrapmodule(joinpath(@__DIR__, "libSubpub"))

function __init__()
    @initcxx
end
end

s = SubPose.Subpub() 
poses = SubPose.get_poses(s)