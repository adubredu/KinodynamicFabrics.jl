using Revise
using KinodynamicFabrics
using KinodynamicFabrics.MuJoCo.PythonCall

# init Digit
visualize = true
prioritize = false
environment = :basketball_env # dodge_env, dodge_hoop_env, basketball_env, cornhole_env, package_env
digit = load_digit(;visualize=visualize, env=environment)

T = 2 # seconds
Horizon = T/digit.Δt # timesteps 

for i=1:Horizon 
    step(digit)
    digit.viewer.render() 
end

digit.viewer.close()