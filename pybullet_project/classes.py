import pybullet as pb
import time
import math
from sympy import symbols, solve, sqrt, Matrix, cos, acos, sin, asin, atan, atan2, simplify, pi, pprint

#Joint Abstraction class
class Joint:
    def __init__(self, bot, index, name, min, max):
        self.bot = bot
        self.index = index
        self.name = name
        self.min = min
        self.max = max
        self.orn = pb.getJointState(self.bot, self.index)[0]
        self.force = 0
        self.position_gain = 0
        self.velocity_gain = 0

    def getPos(self):
        try:
            self.orn = pb.getJointState(self.bot, self.index)[0]
            return self.orn
        except pb.error as e:
            print(f"Joint #{self.index} getPos() fail: {e}")

    def outofrange(self, angle):
        if math.isclose(angle, self.min, abs_tol=1e-4): return False
        if math.isclose(angle, self.max, abs_tol=1e-4): return False
        if angle < self.min or angle > self.max:
            return True
        else:
            return False
        
    def range(self):
        return [self.min, self.max]
    
    def isMax(self):
        return abs(self.max-self.orn) < 0.01
    
    def isMin(self):
        return abs(self.min-self.orn) < 0.01
    
    def setForcePosVelGain(self, force, pos, vel):
        self.force = force
        self.position_gain = pos
        self.velocity_gain = vel
    
    def move(self, amount):
        tgt = self.getPos()
        if (self.isMax()):
            # print(f"!!! JOINT {self.index}.set(): maximum({self.max}) reached")
            pass
        if (self.isMin()):
            # print(f"!!! JOINT {self.index}.set(): minimum reached")
            pass
        tgt = self.getPos() + amount
        pb.setJointMotorControl2(self.bot, self.index, pb.POSITION_CONTROL, 
                                targetPosition=tgt, 
                                force=self.force,
                                positionGain=self.position_gain,
                                velocityGain=self.velocity_gain)
        # if amount != 0: print(f"!!! JOINT {self.index} moving by {amount}")
    
    def set(self, target):
        tgt = 0
        if target and target > self.max:
            # print(f"!!! JOINT {self.index}.set(): maximum({self.max}) reached")
            tgt = self.max
        elif target and target < self.min:
            # print(f"!!! JOINT {self.index}.set(): minimum({self.min}) reached")
            tgt = self.min
        elif target:
            tgt = target
        pb.setJointMotorControl2(self.bot, self.index, pb.POSITION_CONTROL, targetPosition=tgt)

#Link Abstraction class
class Link:
    def __init__(self, prev_joint, next_joint, x_length, x_twist, z_offset, theta):
        self.prev_joint = prev_joint
        self.next_joint = next_joint
        self.x_length = x_length
        self.x_twist = x_twist
        self.z_offset = z_offset
        self.theta = theta
    
    def matrix(self, input=None):
        if input == None:
            theta = self.theta
            Rz=Matrix([
                [cos(theta), -sin(theta), 0, 0],
                [sin(theta), cos(theta), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
        else:
            theta=input
            Rz=Matrix([
                [round(cos(theta), 5), round(-sin(theta), 5), 0, 0],
                [round(sin(theta), 5), round(cos(theta), 5), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
        alpha = self.x_twist
        a = self.x_length
        d = self.z_offset
        Rx=Matrix([
            [1, 0, 0, 0],
            [0, round(cos(alpha), 2), round(-sin(alpha), 2), 0],
            [0, round(sin(alpha), 2), round(cos(alpha), 2), 0],
            [0, 0, 0, 1]
        ])
        Tx=Matrix([
            [1, 0, 0, a],
            [0, 1, 0, 0],
            [0, 0, 1, 0], 
            [0, 0, 0, 1]
        ])

        Tz=Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])
        return Rz*Tz*Tx*Rx

class Bot:
    def __init__(self, name, joints, links):
        self.name = name
        self.joints = joints
        self.links = links
    
    def end_effector_frame(self):
        sol = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
              self.links[0].matrix(self.joints[0].getPos())*\
              self.links[1].matrix(self.joints[1].getPos())*\
              self.links[2].matrix(self.joints[2].getPos())*\
              self.links[3].matrix(self.joints[3].getPos())*\
              self.links[4].matrix(self.joints[4].getPos())*\
              self.links[5].matrix(self.joints[5].getPos())
        return sol

    def solveIK(self, goal):
        # TRIG SOLVE INVERSE KINEMATICS
        adj_goal = goal * \
            Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -150], [0, 0, 0, 1]])
        pos_goal=[round(adj_goal[0, 3], 2), round(adj_goal[1, 3], 2), round(adj_goal[2, 3], 2)] # 3x1 position matrix

        # print("ADJUSTED GOAL FRAME:")
        # pprint(adj_goal)

        # print("POSITION GOAL:", pos_goal)
        def pos_solutions():
            def solveTrig_3(self, goal_x, goal_y, goal_z, theta1_solutions):
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
                    if self.joints[2].outofrange(actual_sol):
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
                        if self.joints[2].outofrange(actual_sol):
                            pass
                        else:
                            output.append(actual_sol)
                    
                    return output
            def solveTrig_2(self, goal_x, goal_y, goal_z, theta3_solutions):
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
                    if self.joints[1].outofrange(solve(expr_3, sol2)[0]-pi): 
                        # print("ALT THETA2 OUT OF RANGE")
                        None
                    else:
                        output.append(solve(expr_3, sol2)[0]-pi)

                # print("theta2 solutions:", output)
                return output

            goal_x=pos_goal[0]
            goal_y=pos_goal[1]
            goal_z=pos_goal[2]

            # print("THETA1 SOLVING.............................")
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
            # print("theta1 solutions:", theta1_solutions)
            theta3_solutions = solveTrig_3(self, goal_x, goal_y, goal_z, theta1_solutions)
            theta2_solutions = solveTrig_2(self, goal_x, goal_y, goal_z, theta3_solutions)

            output = []
            for i in range(0, min(len(theta1_solutions), len(theta2_solutions), len(theta3_solutions))):
                output.append([theta1_solutions[i], theta2_solutions[i], theta3_solutions[i]])
            return output
        
        def rot_solutions(pos_matrix):
            t1 = pos_matrix[0]
            t2 = pos_matrix[1]
            t3 = pos_matrix[2]

            sliced_global_rot = Matrix([adj_goal.row(0)[:3], adj_goal.row(1)[:3], adj_goal.row(2)[:3]])
            sofar_123 = self.links[0].matrix(t1)*self.links[1].matrix(t2)*self.links[2].matrix(t3)
            sofar_123_sliced = Matrix([sofar_123.row(0)[:3], sofar_123.row(1)[:3], sofar_123.row(2)[:3]])
            rot_goal = sofar_123_sliced.T*sliced_global_rot

            theta5 = []
            if not self.joints[4].outofrange(acos(rot_goal[2, 2])):
                theta5 = [acos(rot_goal[2, 2])]
            if len(theta5)>0 and not self.joints[4].outofrange(-theta5[0]) and (-theta5[0] != theta5[0]):
                theta5.append(round(-theta5[0], 5))

            theta6 = []
            if rot_goal[2, 0] == 0 or rot_goal[2, 1] == 0: 
                theta6 = [0]
            elif not self.joints[5].outofrange(-atan2(rot_goal[2, 1], rot_goal[2, 0])):
                theta6 = [atan2(rot_goal[2, 1], rot_goal[2, 0])]
            if theta6!=None and not self.joints[5].outofrange(theta6[0]+pi):
                theta6.append(round(theta6[0]+pi, 4))
                if not self.joints[5].outofrange(-(theta6[0]+pi)): theta6.append(-round(theta6[0]+pi, 4))
            if theta6!=None and not self.joints[5].outofrange(theta6[0]-pi): 
                theta6.append(round(theta6[0]-pi, 4))
                if not self.joints[5].outofrange(-(theta6[0]-pi)): theta6.append(-round(theta6[0]-pi, 4))
            theta6.append(round(-theta6[0], 4))

            theta4 = []
            if rot_goal[0, 2] == 0 or rot_goal[1, 2] == 0: 
                theta4 = [0]
            elif not self.joints[3].outofrange(atan2(rot_goal[1, 2], rot_goal[0, 2])):
                theta4 = [atan2(rot_goal[1, 2], rot_goal[0, 2])]
            if theta4!=None and not self.joints[3].outofrange(theta4[0]+pi):
                theta4.append(round(theta4[0]+pi, 4))
                if not self.joints[3].outofrange(-(theta4[0]+pi)): theta4.append(-round(theta4[0]+pi, 4))
            if theta4!=None and not self.joints[3].outofrange(theta4[0]-pi): 
                theta4.append(round(theta4[0]-pi, 4))
                if not self.joints[3].outofrange(-(theta4[0]-pi)): theta4.append(-round(theta4[0]-pi, 4))
            theta4.append(round(-theta4[0], 4))

            return [theta4,
                    theta5, 
                    theta6]

        ps = pos_solutions()
        pos_rot_solutions = []

        for position in ps:
            rs = rot_solutions(position)
            pos_rot_solutions.append({"pos":position, "rot":rs})

        # FINAL DOUBLE CHECK, APPLY CORRECTIONS
        def solution_legit(t1, t2, t3, t4, t5, t6):
            if t4 == None or t5 == None or t6 == None:
                return False
            # print(f"TRYING SOLUTION: {t1}, {t2}, {t3}, {t4}, {t5}, {t6}")
            theoretical_matrix = Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])*\
                                self.links[0].matrix(t1) * self.links[1].matrix(t2) * self.links[2].matrix(t3) * self.links[3].matrix(t4) * self.links[4].matrix(t5) * self.links[5].matrix(t6)
            # CLEANUP THEORETICAL
            for i in range(0,3):
                for j in range(0,3):
                    val = theoretical_matrix[i, j]
                    theoretical_matrix[i, j] = round(val, 5)
                    if abs(theoretical_matrix[i, j]) < 0.01: theoretical_matrix[i, j] = 0
                    if abs(abs(theoretical_matrix[i, j]) - 1) < 0.01: theoretical_matrix[i, j] = theoretical_matrix[i, j]/abs(theoretical_matrix[i, j])
                    # print(f"cleaning up ROT_GOAL...{i}, {j} set to {rot_goal[i, j]}")

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
                            # print("MARGIN OF ERROR TOO LARGE")
                            return False
            # print("SOLUTION FOUND!!!!!!!")
            return True

        final_output = []
        existing_sols = set()

        # ⭐️⭐️⭐️⭐️ OPTIMIZE ALGORITHM FOR CHECKING ⭐️⭐️⭐️⭐️
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
    
    def moveTo(self, config):
        for i in range(0, len(self.joints)):
            try:
                self.joints[i].move(config[i] - self.joints[i].getPos())
            except:
                pass
        time.sleep(0.01)

    def query(self):
        return [{
                'goal_frame':self.end_effector_frame().tolist(),
                'joint0':round(self.joints[0].getPos(), 4),
                'joint1':round(self.joints[1].getPos(), 4),
                'joint2':round(self.joints[2].getPos(), 4),
                'joint3':round(self.joints[3].getPos(), 4),
                'joint4':round(self.joints[4].getPos(), 4),
                'joint5':round(self.joints[5].getPos(), 4)
            }]

# FIND SOME WAY TO 

# Add some sort of joint correction; bending shoulder forward too far when elbow is fully bent can cause collision
# Note & teach robot safe zones for each joint to prevent collisions that happen based on certain joint orientations
#   EX1) wrist colliding with base when elbow fully bent and shoulder bends forward
#   EX2) palm colliding with base when wrist bent inward and elbow is bent inward too