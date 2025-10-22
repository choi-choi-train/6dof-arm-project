import pybullet as pb
import pybullet_data, math, time, os, ast
import pandas
from sympy import symbols, Matrix, cos, sin, simplify, pi, pprint
from classes import Joint, Link, Bot

# ======================================= SETUP CODE ==================================
physics_client = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.resetSimulation()

plane = pb.loadURDF("plane.urdf")
bot = pb.loadURDF("6DOF_junior/model.urdf", [0,0,0], useFixedBase=1)
pb.addUserDebugLine([0,0,0], [1,0,0], [100,0,0], lineWidth=20, lifeTime=0)
pb.addUserDebugLine([0,0,0], [0,1,0], [0,100,0], lineWidth=20, lifeTime=0)
pb.addUserDebugLine([0,0,0], [0,0,1], [0,0,100], lineWidth=20, lifeTime=0)

orientation = None

joints=[]
for i in range(0, pb.getNumJoints(bot)):
    joint_info = pb.getJointInfo(bot, i) #gets joint info list for joint 2
    name, type, lower_limit, upper_limit = \
        joint_info[1], joint_info[2], joint_info[8], joint_info[9]
    
    if (type == 0):
        print(f"JOINT#{i}: {name} {round(math.degrees(lower_limit), 2)}deg ~ {round(math.degrees(upper_limit), 2)}deg")
        new_joint = Joint(bot, i, name, lower_limit, upper_limit)
        joints.append(new_joint)

links=[]
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
final_link = Link(joints[5], joints[5], links_Tx[5], links_Rx[5], links_Tz[5], links_Rz[5])
links.append(final_link)

junior = Bot('junior', joints, links)

pb.setGravity(0, 0, 0)
pb.setRealTimeSimulation(1)

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

# ====================================== THE GOOD STUFF ================================
all_sols = []

start = time.perf_counter()
if os.path.getsize('sequence.csv') == 0:
    print("ERROR: CSV IS COMPLETELY EMPTY")
elif pandas.read_csv('sequence.csv').empty:
    print("ERROR: CSV HAS NO DATA")
else:
    data = pandas.read_csv('sequence.csv')
    frames = data['goal_frame'].tolist()
    print(f"{len(frames)} frames to evaluate!")

    for i in range(len(frames)):
        f = Matrix(ast.literal_eval(frames[i]))
        sol = junior.solveIK(f)
        all_sols.append(sol)

opt_sols = []
first_pos = [0, -pi/2, 0, 0, 0, 0]
for i in range(0, len(all_sols)): 
    set = all_sols[i]
    closest = set[0]
    closest_dist = 10000000000
    print(f"⭐️ FRAME {i}: processing {len(set)} configurations")
    for config in set:
        t1, t2, t3, t4, t5, t6 = config[0], config[1], config[2], config[3], config[4], config[5]
        # print(f"evaluating config: {t1} {t2} {t3} {t4} {t5} {t6}")
        if i == 0:
            i1, i2, i3, i4, i5, i6 = 0, -pi/2, 0, 0, 0, 0
        else:
            i1, i2, i3, i4, i5, i6 = opt_sols[i-1][0], opt_sols[i-1][1], opt_sols[i-1][2], opt_sols[i-1][3], opt_sols[i-1][4], opt_sols[i-1][5]
        
        # weights; put maximum weight on big-stress joint, minimum weight on small easy joints; idk if this really matters but it sounds intuitive
        # 25% on base rot & elbow bend
        # 40% on shoulder bend
        # 3% on forearm swivel, 3% on wrist swivel, 4% on wrist bend
        dist_algo = 0.25*abs(t1-i1) + 0.4*abs(t2-i2) + 0.25*abs(t3-i3) + 0.03*abs(t4-i4) + 0.04*abs(t5-i5) + 0.03*abs(t6-i6)
        if dist_algo < closest_dist:
            # print(f"🤖 CLOSER ANSWER FOUND, {dist_algo} < {closest_dist}")
            closest = config
            closest_dist = dist_algo

    opt_sols.append(closest)

print(f"OPTIMUM SOLUTION SEQUENCE, {len(opt_sols)} STEPS:")
print(opt_sols)

start = time.perf_counter()
curr = 0

while True:
    keys = pb.getKeyboardEvents()
    elapsed_time = round((time.perf_counter()-start), 0)

    junior.moveTo(opt_sols[curr])

    if elapsed_time > 5 and elapsed_time > 5 * (curr+1) and curr < len(opt_sols) - 1:
        curr += 1
        print(f"TIME: {elapsed_time} -- NEXT POSITION, POSITION #{curr}")

    pb.stepSimulation()
    time.sleep(1./240.)