using MuJoCo 
using MuJoCo.PythonCall

xml_path = "src/model/gripper_scene.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data,  hide_menus=true)
for i=1:1000000
    mujoco.mj_step(model, data) 
    viewer.render() 
end
viewer.close() 
