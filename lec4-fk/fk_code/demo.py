# Franka Emika Panda MuJoCo Simulation Demo
import mujoco
import mujoco.viewer
import os

# Path to the XML model file
MODEL_PATH = os.path.join(os.path.dirname(__file__), "franka_emika_panda", "scene.xml")

# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Set all geoms to be semi-transparent (e.g., alpha=0.4)
for i in range(model.ngeom):
    model.geom_rgba[i][3] = 0.4  # Set alpha to 0.4 for transparency

# Launch the viewer and run the simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
    print("Press ESC in the viewer window to exit.")
    while viewer.is_running():
        mujoco.mj_step(model, data)
        # Print the world position and orientation matrix of each body (FK of each frame)
        print("--- Forward Kinematics (World Position & Orientation) of Each Frame ---")
        for i in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
            pos = data.xpos[i]  # World position of the origin of the body frame
            mat = data.xmat[i].reshape(3, 3)  # 3x3 rotation matrix
            print(f"{name}: position = [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            print(f"{name}: orientation matrix =\n{mat}")
        print("--------------------------------------------------------")
        viewer.sync()
