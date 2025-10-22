import pybullet as pb
import pybullet_data
import time
from classes import Joint, Link
import math
from sympy import symbols, Matrix, cos, sin, simplify, pprint, pi

# os.system("git clone https://github.com/ros-industrial/kuka_experimental.git")
physics_client = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.resetSimulation()

plane = pb.loadURDF("plane.urdf")
bot = pb.loadURDF("6DOF_junior/model.urdf", [0,0,0], useFixedBase=1)
orientation = None

#ROBOT INFORMATION
print("6DOF JUNIOR ROBOT:", pb.getNumJoints(bot), "JOINTS")
#getJointInfo: [jointIndex, jointName, jointType, qIndex, uIndex, flags, jointDamping, jointFriction,
#               jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity, linkName, jointAxis,
#               parentFramePos, parentFrameOrn, parentIndex]
joints=[]
for i in range(0, pb.getNumJoints(bot)):
    joint_info = pb.getJointInfo(bot, i) #gets joint info list for joint 2
    name, type, lower_limit, upper_limit = \
        joint_info[1], joint_info[2], joint_info[8], joint_info[9]
    
    if (type == 0):
        print(f"joint created: #{i} {name} {round(math.degrees(lower_limit), 2)}deg ~ {round(math.degrees(upper_limit), 2)}deg")
        new_joint = Joint(bot, i, name, lower_limit, upper_limit)
        joints.append(new_joint)
    #0: revolute, 1: prismatic, 2: spherical, 3: planar, 4: fixed

joint_positions = [j.getPos() for j in joints]
print("JOINT POSITIONS:", joint_positions)

pb.setGravity(0, 0, 0)
pb.setRealTimeSimulation(1)
#ARGS: setJointMotorControlArray(
    #     bodyUniqueId,
    #     jointIndices,
    #     controlMode,                POSITION_CONTROL, VELOCITY_CONTROL, TORQUE_CONTROL
    #     targetPositions=None,
    #     targetVelocities=None,
    #     forces=None,
    #     positionGains=None,
    #     velocityGains=None
    # )
#ARGS: setJointMotorControl2(
    #     bodyUniqueId,
    #     jointIndex,
    #     controlMode,                POSITION_CONTROL, VELOCITY_CONTROL, TORQUE_CONTROL
    #     targetPosition=None,
    #     targetVelocity=None,
    #     force=500,
    #     positionGain=0.1,
    #     velocityGain=1.0,
    #     maxVelocity=None
    # )

start = time.perf_counter()

# PROGRAM ACTIONS IN SEQUENCE HERE
sequence_queue = [
    (1, lambda: joints[0].set(-2.1571)),
    (1, lambda: joints[1].set(-3.0105)),
    (1, lambda: joints[2].set(0)),
    (1, lambda: joints[3].set(-3.1391)),
    (1, lambda: joints[4].set(-1.5613)),
    (1, lambda: joints[5].set(0)),
    # (2, lambda: joints[2].move(pi/6)),
    # (3, lambda: joints[3].move(pi/6)),
    # (4, lambda: joints[4].move(pi/6)),
    # (5, lambda: joints[5].move(pi/6)),
    # (7, lambda: [(j.set(0)) for j in joints]),
    # (7, lambda: joints[2].set(-pi/2)),
    # (8, lambda: [print("LINK POSITIONS:", [pb.getLinkState(bot, l)[:2][0] for l in range(6)])])
]
print(f"starting sequence...{len(sequence_queue)} actions to be performed")
tuple = sequence_queue[0]
action_time = tuple[0]
action_func = tuple[1]

# FORCE, POSITION GAIN, VELOCITY GAIN OF JOINTS
joints[0].setForcePosVelGain(900, 0.005, 1) # shoulder swivel
joints[1].setForcePosVelGain(900, 0.005, 1) # shoulder bend
joints[2].setForcePosVelGain(900, 0.01, 1) # elbow bend
joints[2].set(0)
joints[3].setForcePosVelGain(900, 0.01, 1) # forearms swivel
joints[4].setForcePosVelGain(900, 0.01, 1) # wrist bend
joints[5].setForcePosVelGain(900, 0.01, 1) # wrist swivel

# SET STARTING POSITIONS
joints[0].set(0)
joints[1].set(-pi/2)
joints[2].set(0)
joints[3].set(0)
joints[4].set(0)
joints[5].set(0)

while True:
    elapsed_time = round((time.perf_counter()-start), 2)
    # print(elapsed_time)
    if (elapsed_time > action_time):
        print(f"{action_time}s elapsed, performing action")
        action_func()
        if (len(sequence_queue) > 1):
            sequence_queue.pop(0)
            new_tuple = sequence_queue[0]
            action_time = new_tuple[0]
            action_func = new_tuple[1]
        else:
            action_time = math.inf
            action_func = None
    pb.stepSimulation()
    time.sleep(1./240.)