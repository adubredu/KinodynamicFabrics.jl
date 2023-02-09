using VideoIO
# using MuJoCo.PythonCall

images = []
for i in eachindex(pixels)
    img = pyconvert(Vector{Vector{Vector{UInt8}}}, pixels[i])
    @show i
    push!(images, img)
end
