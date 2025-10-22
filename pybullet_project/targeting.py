import pybullet as pb
import pybullet_data
import time
from sympy import symbols, solve, Matrix, simplify, pprint, pi, Eq, trigsimp, \
    sin, cos, tan, asin, acos, atan, atan2, sqrt
from classes import Joint, Link, Bot
import math

# os.system("git clone https://github.com/ros-industrial/kuka_experimental.git")
physics_client = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.resetSimulation()

plane = pb.loadURDF("plane.urdf")
bot = pb.loadURDF("6DOF_junior/model.urdf", [0,0,0], useFixedBase=1)
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
        print(f"JOINT#{i}: {name} {round(math.degrees(lower_limit), 2)}deg ~ {round(math.degrees(upper_limit), 2)}deg")
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
    # print(f"LINK#{i}: {new_link.prev_joint.name}~{new_link.next_joint.name}, x-dist. {new_link.x_length}, x_twist. {new_link.x_twist}, z_offset {new_link.z_offset}, {new_link.theta}")
final_link = Link(joints[5], joints[5], links_Tx[5], links_Rx[5], links_Tz[5], links_Rz[5])
links.append(final_link)
# print(f"FINAL LINK(END-EFFECTOR): {final_link.prev_joint.name}~{final_link.next_joint.name}, x-dist. {final_link.x_length}, x_twist. {final_link.x_twist}, z_offset {final_link.z_offset}, {final_link.theta}")

# print("--------------------------------")
# print("TRANSFORMATION MATRICES:")
# for i in range(0, pb.getNumJoints(bot)):
#     print(f"LINK {i} MATRIX")
#     pprint(links[i].matrix())
junior = Bot('junior', joints, links)

pb.setGravity(0, 0, 0)
pb.setRealTimeSimulation(1)

# EXAMPLE POSITION 1, calculated using FK from base frame
goal_frame = Matrix([[0.867527025261093, 0.479209792733087, 0.133219965090582, 293.181326534097], [-0.483520030975342, 0.875329971313477, 0, 0], [-0.116611754608665, -0.0644147019390454, 0.991085871530231, 534.211247319251], [0, 0, 0, 1]])

# TRIG SOLVE INVERSE KINEMATICS
def solveIK_Trig(goal): 
    adj_goal = goal * \
    Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -150], [0, 0, 0, 1]])
    pos_goal=[round(adj_goal[0, 3], 2), round(adj_goal[1, 3], 2), round(adj_goal[2, 3], 2)] # 3x1 position matrix

    print("ADJUSTED GOAL FRAME:")
    pprint(adj_goal)

    print("POSITION GOAL:", pos_goal)
    def pos_solutions():
        goal_x=pos_goal[0]
        goal_y=pos_goal[1]
        goal_z=pos_goal[2]

        print("THETA1 SOLVING.............................")
        if goal_x != 0:
            theta1_solutions = [atan2(goal_y, goal_x)]
        else: 
            theta1_solutions = [0]
        if abs(goal_y)<0.05 and goal_x > 0:
            theta1_solutions = [0]
        elif abs(goal_y)<0.05 and goal_x < 0:
            theta1_solutions = [pi]

        if (theta1_solutions[0] - pi >= -pi): theta1_solutions.append(theta1_solutions[0]-pi)
        if (theta1_solutions[0] + pi <= pi): theta1_solutions.append(theta1_solutions[0]+pi)
        print("theta1 solutions:", theta1_solutions)
        theta3_solutions = solveTrig_3(goal_x, goal_y, goal_z, theta1_solutions)
        theta2_solutions = solveTrig_2(goal_x, goal_y, goal_z, theta3_solutions)

        output = []
        for i in range(0, min(len(theta1_solutions), len(theta2_solutions), len(theta3_solutions))):
            output.append([theta1_solutions[i], theta2_solutions[i], theta3_solutions[i]])
        return output
    
    def rot_solutions(pos_matrix):
        t1 = pos_matrix[0]
        t2 = pos_matrix[1]
        t3 = pos_matrix[2]

        sliced_global_rot = Matrix([adj_goal.row(0)[:3], adj_goal.row(1)[:3], adj_goal.row(2)[:3]])
        sofar_123 = links[0].matrix(t1)*links[1].matrix(t2)*links[2].matrix(t3)
        sofar_123_sliced = Matrix([sofar_123.row(0)[:3], sofar_123.row(1)[:3], sofar_123.row(2)[:3]])
        rot_goal = sofar_123_sliced.T*sliced_global_rot

        theta5 = []
        if not joints[4].outofrange(acos(rot_goal[2, 2])):
            theta5 = [acos(rot_goal[2, 2])]
        if len(theta5)>0 and not joints[4].outofrange(-theta5[0]) and (-theta5[0] != theta5[0]):
            theta5.append(round(-theta5[0], 5))

        theta6 = []
        if rot_goal[2, 0] == 0 or rot_goal[2, 1] == 0: 
            theta6 = [0]
        elif not joints[5].outofrange(-atan2(rot_goal[2, 1], rot_goal[2, 0])):
            theta6 = [atan2(rot_goal[2, 1], rot_goal[2, 0])]
        if theta6!=None and not joints[5].outofrange(theta6[0]+pi):
            theta6.append(round(theta6[0]+pi, 4))
            if not joints[5].outofrange(-(theta6[0]+pi)): theta6.append(-round(theta6[0]+pi, 4))
        if theta6!=None and not joints[5].outofrange(theta6[0]-pi): 
            theta6.append(round(theta6[0]-pi, 4))
            if not joints[5].outofrange(-(theta6[0]-pi)): theta6.append(-round(theta6[0]-pi, 4))
        theta6.append(round(-theta6[0], 4))

        theta4 = []
        if rot_goal[0, 2] == 0 or rot_goal[1, 2] == 0: 
            theta4 = [0]
        elif not joints[3].outofrange(atan2(rot_goal[1, 2], rot_goal[0, 2])):
            theta4 = [atan2(rot_goal[1, 2], rot_goal[0, 2])]
        if theta4!=None and not joints[3].outofrange(theta4[0]+pi):
            theta4.append(round(theta4[0]+pi, 4))
            if not joints[3].outofrange(-(theta4[0]+pi)): theta4.append(-round(theta4[0]+pi, 4))
        if theta4!=None and not joints[3].outofrange(theta4[0]-pi): 
            theta4.append(round(theta4[0]-pi, 4))
            if not joints[3].outofrange(-(theta4[0]-pi)): theta4.append(-round(theta4[0]-pi, 4))
        theta4.append(round(-theta4[0], 4))
        
        print(f"THETA4:", theta4)
        print(f"THETA5:", theta5)
        print(f"THETA6:", theta6)

        return [theta4,
                theta5, 
                theta6]

    print("🧠 CALCULATING POSITIONS =================================================")
    ps = pos_solutions()
    print(f"!!!!!POS SOLUTIONS ({len(ps)}): {ps} \n")
    pos_rot_solutions = []
    
    print("🧠 EVALUATING POSITIONS IN POS_SOLUTIONS() =======================================")
    for position in ps:
        print("~ SOLUTION......")
        rs = rot_solutions(position)
        print("ROTATION SOLUTIONS:", rs)
        pos_rot_solutions.append({"pos":position, "rot":rs})

    print("\n")

    # FINAL DOUBLE CHECK, APPLY CORRECTIONS
    def solution_legit(t1, t2, t3, t4, t5, t6):
        if t4 == None or t5 == None or t6 == None:
            return False
        print(f"~~ TRYING SOLUTION: [{t1}, {t2}, {t3}, {t4}, {t5}, {t6}]")
        theoretical_matrix = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
                             links[0].matrix(t1) * links[1].matrix(t2) * links[2].matrix(t3) * links[3].matrix(t4) * links[4].matrix(t5) * links[5].matrix(t6)
        # CLEANUP THEORETICAL
        for i in range(0,3):
            for j in range(0,3):
                val = theoretical_matrix[i, j]
                theoretical_matrix[i, j] = round(val, 5)
                if abs(theoretical_matrix[i, j]) < 0.01: theoretical_matrix[i, j] = 0
                if abs(abs(theoretical_matrix[i, j]) - 1) < 0.01: theoretical_matrix[i, j] = theoretical_matrix[i, j]/abs(theoretical_matrix[i, j])

        # print("THEORETICAL MATRIX:")
        # pprint(theoretical_matrix)

        for i in range(0, 3):
            for j in range(0, 4):
                theo = theoretical_matrix[i, j]
                gol = goal[i, j]
                if theo == 0 and gol == 0: continue
                if theo != 0 and gol == 0:
                    if abs(theo - gol) > 0.001: 
                        # print("THEO!=0 GOL==0: MARGIN OF ERROR TOO LARGE")
                        return False
                if theo != 0 and gol != 0:
                    if j == 3 and theo - gol < 0.5:
                        continue
                    if abs((theo - gol)/gol) > 0.01:
                        # print(f"COMPARING {theo} and {gol}..........")
                        # print("MARGIN OF ERROR TOO LARGE")
                        return False
        print("🎉 SOLUTION FOUND")
        return True

    final_output = []
    existing_sols = set()
    
    print("🧠 CHECKING POS_ROT COMBINATIONS ====================================================")
    for posrot in pos_rot_solutions:
        p = posrot["pos"]
        r = posrot["rot"]

        t4, t5, t6 = r[0], r[1], r[2]
        
        for t4_sol in t4:
            for t5_sol in t5:
                for t6_sol in t6:
                    str_encode = str([round(p[0], 4), round(p[1], 4), round(p[2], 4), round(t4_sol, 4), round(t5_sol, 4), round(t6_sol, 4)])
                    if str_encode not in existing_sols:
                        existing_sols.add(str_encode)
                        if solution_legit(p[0], p[1], p[2], t4_sol, t5_sol, t6_sol):
                            final_output.append([round(p[0], 4), round(p[1], 4), round(p[2], 4), round(t4_sol, 4), round(t5_sol, 4), round(t6_sol, 4)])
        
    return final_output

def solveTrig_3(goal_x, goal_y, goal_z, theta1_solutions):
    sol3 = symbols('sol3')
    wrist_elbow_ang = atan(350/81.5)
    forearm_len = sqrt(81.5**2 + 350**2)
    upper_arm_len = 300
    output = []

    xy_diag = sqrt((goal_x**2)+goal_y**2)-50.41
    shoulder_wrist_dist = sqrt(xy_diag**2 + goal_z**2)

    if (shoulder_wrist_dist > 659.36):
        return output
    elif shoulder_wrist_dist > 650:
        output.append(-pi/2)
    else:
        c_squared = shoulder_wrist_dist**2
        b_squared = upper_arm_len**2
        a_squared = forearm_len**2
        expr_2 = acos((c_squared-a_squared-b_squared)/(-2*forearm_len*upper_arm_len))-sol3
        sol = solve(expr_2, sol3)
        actual_sol = -(wrist_elbow_ang - (pi - sol[0]))
        if joints[2].outofrange(actual_sol):
            pass
        else:
            output.append(actual_sol)
    
    if (len(theta1_solutions) > 1):
        xy_diag_alt = sqrt((goal_x**2)+goal_y**2)+50.41
        shoulder_wrist_dist_alt = sqrt(xy_diag_alt**2 + goal_z**2)

        if (shoulder_wrist_dist_alt > 659.36):
            return output
        elif shoulder_wrist_dist_alt > 650:
            output.append(-pi/2)
        else:
            c_squared = shoulder_wrist_dist_alt**2
            b_squared = upper_arm_len**2
            a_squared = forearm_len**2
            expr_2 = acos((c_squared-a_squared-b_squared)/(-2*forearm_len*upper_arm_len))-sol3
            sol = solve(expr_2, sol3)
            actual_sol = -(wrist_elbow_ang - (pi-sol[0]))
            if joints[2].outofrange(actual_sol):
                pass
            else:
                output.append(actual_sol)
        
        return output
    
def solveTrig_2(goal_x, goal_y, goal_z, theta3_solutions):
    # print("THETA2 SOLVING.............................")
    sol2 = symbols('sol2')
    output = []

    wrist_upperarm_ang = (atan(81.5/350)+pi/2)-theta3_solutions[0]
    # print("wrist_upperarm_ang =", wrist_upperarm_ang)
    xy_diag = sqrt((goal_x**2)+goal_y**2)-50.41
    shoulder_wrist_dist = sqrt(xy_diag**2 + goal_z**2)
    expr_3 = atan2(goal_z, xy_diag)+asin((359.3636*sin(wrist_upperarm_ang))/shoulder_wrist_dist)-sol2
    output.append(-1*solve(expr_3, sol2)[0])

    if (len(theta3_solutions) > 1):
        # print("WORKING ON ALT THETA2.....")
        wrist_upperarm_ang = theta3_solutions[1]+(atan(350/81.5)+pi)
        xy_diag = sqrt((goal_x**2)+goal_y**2)+50.41
        shoulder_wrist_dist = sqrt(xy_diag**2 + goal_z**2)
        expr_3 = atan2(goal_z, xy_diag)+asin((359.3636*sin(wrist_upperarm_ang))/shoulder_wrist_dist)-sol2
        if joints[1].outofrange(solve(expr_3, sol2)[0]-pi): 
            # print("ALT THETA2 OUT OF RANGE")
            None
        else:
            output.append(solve(expr_3, sol2)[0]-pi)

    # print("theta2 solutions:", output)
    return output

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

sol = solveIK_Trig(goal_frame)
print(f"{len(sol)} SOLUTIONS FOUND, CYCLING THROUGH............")
print(sol)

start = time.perf_counter()
curr = 0
print(f"TIME: 0 -- FIRST POSITION, POSITION #0")

while True:
    keys = pb.getKeyboardEvents()
    elapsed_time = round((time.perf_counter()-start), 0)

    junior.moveTo(sol[curr])
    
    if elapsed_time > 5 and elapsed_time > 5 * (curr+1) and curr < len(sol) - 1:
        curr += 1
        print(f"TIME: {elapsed_time} -- NEXT POSITION, POSITION #{curr}")

    pb.stepSimulation()
    time.sleep(1./240.)

#TODOS:
# 2) Learn about workspace & dextrous workspace, and figure out ways to check for these spaces
# 3) Figure out how to find best-fit orientation
# 4) Trim down redundancies, figure out shortest-path movements to create sequence