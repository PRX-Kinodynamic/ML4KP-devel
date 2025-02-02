import sys
import mujoco
import mujoco.viewer
# sys.path.append('/home/dhruv/2024/projects/ml4kp_ktamp/lib')
import PyML4KP as prx





# Initialize MuJoCo environment
m = mujoco.MjModel.from_xml_path('/home/dhruv/2024/projects/ml4kp_ktamp/resources/models/mujoco_envs/env_config_1.xml')
data = mujoco.MjData(m)

# 

# print(state_memory.mem_ptr)

print(m.nq, m.nv, m.nu, m.njnt)

print(len(m.jnt_limited))

state_topology = ""
for i in range(m.njnt):
    if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_SLIDE:
        print(m.names[i])
        state_topology += "E"
    
    if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
        state_topology += "EEEQQQQ"
    
for i in range(m.nv):

    state_topology += "E";


state_memory = prx.space_memory(m.nq + m.nv)
state_space = prx.space_t(state_topology, state_memory, "state_space")
print(state_space.get_dimension())

control_topology = "E"*m.nu
control_memory = prx.space_memory(m.nu)
control_space = prx.space_t(control_topology, control_memory, "control_space")
print(control_space.get_dimension())
# TODO:set bounds


mujoco_system = prx.system("mujoco_system")
mujoco_system.set_state_space(state_space)
mujoco_system.set_input_control_space(control_space)

system_group = prx.system_group([mujoco_system])

print(system_group.get_state_space())
print(system_group.get_control_space())















    # if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
    #     print(m.names[i], m.jnt_type[i], m.jnt_limited[i], m.jnt_range[i], m.jnt_qposadr[i], m.jnt_dofadr[i])
    # if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_SLIDE:
    #     print(m.names[i], m.jnt_type[i], m.jnt_limited[i], m.jnt_range[i], m.jnt_qposadr[i], m.jnt_dofadr[i])
    

    
    
    
    # print(m.names[i], m.jnt_type[i], m.jnt_limited[i], m.jnt_range[i], m.jnt_qposadr[i], m.jnt_dofadr[i])


# for i, j in zip(model.names, model.jnt_type):
#     print(i, j)

# control_memory = prx.space_memory(model.nu)



# state_space = prx.space_t("E"*(model.nq + model.nv), state_memory, "state_space")
# control_space = prx.space_t("E"*model.nu, control_memory, "control_space")

# print(state_space.get_dimension())
# print(control_space.get_dimension())





# print(model.nq, model.nv)
# print(data.qpos.shape)
# print(data.qvel.shape)


# # create the space_t state space  for the planner 
# state_space = prx.space_t(model.nq, model.nv)
# 