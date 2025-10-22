import pybullet as pb
import pybullet_data, math, time, os
import pandas
from classes import Joint, Link, Bot
from sympy import symbols, Matrix, cos, sin, simplify, pprint, pi

# os.system("git clone https://github.com/ros-industrial/kuka_experimental.git")
physics_client = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.resetSimulation()

plane = pb.loadURDF("plane.urdf")
bot = pb.loadURDF("/6DOF_junior/model.urdf",
                   [0,0,0], useFixedBase=1)
pb.addUserDebugLine([0,0,0], [1,0,0], [100,0,0], lineWidth=20, lifeTime=0)
pb.addUserDebugLine([0,0,0], [0,1,0], [0,100,0], lineWidth=20, lifeTime=0)
pb.addUserDebugLine([0,0,0], [0,0,1], [0,0,100], lineWidth=20, lifeTime=0)
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

links=[] #shoulder, upper_arm, forearm, wrist, palm, end-effector
# 6 Links, excluding base link since frame is coincident w/ shoulder swivel joint
#USER PRESET
links_Tx = [50.41, 300, 81.5, 0, 0, 0] 
links_Rx = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0] 
links_Tz = [0, 0 ,0, 350, 0, 150]
links_Rz = [symbols('theta1'), symbols('theta2'), symbols('theta3'),
               symbols('theta4'), symbols('theta5'), symbols('theta6')]
for i in range(0, pb.getNumJoints(bot)-1):
    joint_prev = joints[i]
    joint_next = joints[i+1]
    new_link = Link(joint_prev, joint_next, links_Tx[i], links_Rx[i], links_Tz[i], links_Rz[i])
    links.append(new_link)
    print(f"LINK#{i}: {new_link.prev_joint.name}~{new_link.next_joint.name}, x-dist. {new_link.x_length}, x_twist. {new_link.x_twist}, z_offset {new_link.z_offset}, {new_link.theta}")
final_link = Link(joints[5], joints[5], links_Tx[5], links_Rx[5], links_Tz[5], links_Rz[5])
links.append(final_link)
# print(f"FINAL LINK(END-EFFECTOR): {final_link.prev_joint.name}~{final_link.next_joint.name}, x-dist. {final_link.x_length}, x_twist. {final_link.x_twist}, z_offset {final_link.z_offset}, {final_link.theta}")

# joint_positions = [j.getPos() for j in joints]
# print("JOINT POSITIONS:", joint_positions)
# link_positions = [pb.getLinkState(bot, l)[:2][0] for l in range(6)]
# print("LINK POSITIONS:", link_positions)

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

junior = Bot('junior', joints, links)

# HELPER FUNCTION: CONTROL JOINT
def controlJoint(index, neg_key, pos_key, amount):
    if (ord(neg_key) in keys and keys[ord(neg_key)] & pb.KEY_IS_DOWN):
        # Bitwise & screens out pb.KEY_WAS_TRIGGERED (0b011 -> 0b001)
        joints[index].move(-amount)
    if (ord(pos_key) in keys and keys[ord(pos_key)] & pb.KEY_IS_DOWN):
        joints[index].move(amount)
    if (ord(neg_key) in keys and keys[ord(neg_key)] & pb.KEY_WAS_RELEASED) or \
       (ord(pos_key) in keys and keys[ord(pos_key)] & pb.KEY_WAS_RELEASED):
        joints[index].set(joints[index].getPos())

# START SIMULATION
pb.setGravity(0, 0, 0)
pb.setRealTimeSimulation(1)

sequence = []
scene_playing = False
start = time.perf_counter()

while True:
    keys = pb.getKeyboardEvents()
    
    if (ord('1') in keys and keys[ord('1')] & pb.KEY_WAS_TRIGGERED):
        start = time.perf_counter()
        print(f"PLAYING SEQUENCE:", sequence, "TIME:", start)
        scene_playing = True
        counter = 0

    if not scene_playing:
        controlJoint(0, 'n', 'm', pi/2)
        controlJoint(1, 'j', 'k', pi/3)
        controlJoint(2, 'u', 'i', pi/4)
        controlJoint(3, 't', 'y', pi/4)
        controlJoint(4, 'e', 'r', pi/4)
        controlJoint(5, 'd', 'f', pi/2)

        if (ord('`') in keys and keys[ord('`')] & pb.KEY_WAS_TRIGGERED):
            snapshot = []
            for i in range(0, pb.getNumJoints(bot)):
                snapshot.append(round(joints[i].getPos(), 2))
            print("SNAPSHOT!", snapshot)
            sequence.append(snapshot)
        if (ord('q') in keys and keys[ord('q')] & pb.KEY_WAS_TRIGGERED):
            # joint_0_matrix = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
            #                  links[0].matrix(joints[0].getPos())
            # print(f"JOINT #0: {joints[0].getPos()}rad, MATRIX:")
            # pprint(positionMatrix)

            # joint_1_matrix = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
            #                  links[0].matrix(joints[0].getPos())\
            #                  * links[1].matrix(joints[1].getPos())
            # print(f"JOINT #1: {joints[1].getPos()}rad, MATRIX:")
            # pprint(positionMatrix)

            # joint_2_matrix = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
            #                  links[0].matrix(joints[0].getPos())\
            #                  * links[1].matrix(joints[1].getPos())\
            #                  * links[2].matrix(joints[2].getPos())
            # print(f"JOINT #2: {joints[2].getPos()}rad, MATRIX:")
            # pprint(positionMatrix)

            # joint_3_matrix = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
            #                  links[0].matrix(joints[0].getPos())\
            #                  * links[1].matrix(joints[1].getPos())\
            #                  * links[2].matrix(joints[2].getPos())\
            #                  * links[3].matrix(joints[3].getPos())
            # print(f"JOINT #3: {joints[3].getPos()}rad, MATRIX:")
            # pprint(positionMatrix)

            # joint_4_matrix = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
            #                  links[0].matrix(joints[0].getPos())\
            #                  * links[1].matrix(joints[1].getPos())\
            #                  * links[2].matrix(joints[2].getPos())\
            #                  * links[3].matrix(joints[3].getPos())\
            #                  * links[4].matrix(joints[4].getPos())
            # print(f"JOINT #4: {joints[4].getPos()}rad, MATRIX:")
            # pprint(positionMatrix)

            # joint_5_matrix = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
            #                  links[0].matrix(joints[0].getPos())\
            #                  * links[1].matrix(joints[1].getPos())\
            #                  * links[2].matrix(joints[2].getPos())\
            #                  * links[3].matrix(joints[3].getPos())\
            #                  * links[4].matrix(joints[4].getPos())\
            #                  * links[5].matrix(joints[5].getPos())
            # print(f"JOINT #5: {joints[5].getPos()}rad, MATRIX:")
            # pprint(positionMatrix)

            # data = pandas.DataFrame({
            #     'goal_frame':[joint_5_matrix.tolist()],
            #     'joint0':round(joints[0].getPos(), 4),
            #     'joint1':round(joints[1].getPos(), 4),
            #     'joint2':round(joints[2].getPos(), 4),
            #     'joint3':round(joints[3].getPos(), 4),
            #     'joint4':round(joints[4].getPos(), 4),
            #     'joint5':round(joints[5].getPos(), 4)
            # })
            data = pandas.DataFrame(junior.query())
            print('data added to csv')
            print(data)
            if os.path.getsize('sequence.csv') == 0:
                data.to_csv('./sequence.csv', index=False)
            else:
                check = pandas.read_csv('./sequence.csv')
                if check.empty:
                    data.to_csv('./sequence.csv', index=False)
                else:
                    data.to_csv('./sequence.csv', mode='a', header=False, index=False)
    else:
        elapsed_time = round((time.perf_counter()-start), 0)
        print(elapsed_time)
        if elapsed_time > 0 and elapsed_time >= counter*2:
            print(f"PLAYING SHOT {counter+1} of {len(sequence)} SHOTS")
            shot = sequence[counter]
            for i in range(0, pb.getNumJoints(bot)):
                joints[i].set(shot[i])
            counter += 1
        if elapsed_time > 0 and counter >= len(sequence):
            print("SEQUENCE OVER")
            sequence = []
            scene_playing = False

    pb.stepSimulation()
    time.sleep(1./240.)

# TODOS: 
# 2) BONUS: maybe find some way to "intuitively" move the end-effector with WASD or numpad controls(like GTA5 i think) borrowing inverse kinematics from targetin.py
# 3) BONUS BONUS? record all frames somehow then replay them precisely including velocities, pauses, etc. 