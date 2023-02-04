using Revise
using KinodynamicFabrics
using KinodynamicFabrics.MuJoCo.PythonCall

digit = load_digit()
 
for i=1:10000
    if pyconvert(Bool, digit.viewer.is_alive)
        step(digit)
        digit.viewer.render()
    else
        break 
    end
end

digit.viewer.close()

q, qdot, qmotors = get_generalized_coordinates(digit)



#= 
5.2kHz data reception speed
=#