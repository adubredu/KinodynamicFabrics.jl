using MuJoCo
using MuJoCo.PythonCall

xml_path = joinpath(@__DIR__, "../src/model/scene.xml")

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

for i=1:10000
    if pyconvert(Bool, viewer.is_alive)
        mujoco.mj_step(model, data)
        viewer.render()
    else
        break 
    end
end

viewer.close()