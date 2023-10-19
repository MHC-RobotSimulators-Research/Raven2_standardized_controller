import time
from ambf_client import Client
import math
import raven_ik as ik
import raven_fk as fk
import utilities as u
import numpy as np
import ambf_raven_def as ard
import timeit


'''
author: Natalie Chalfant, Sean Fabrega
ambf_raven defines methods for an ambf_raven robot, including homnig, sine dance and hoping soon
cube tracing and soft body manipulation'''


class ambf_raven:
    def __init__(self):
        self._client = Client()
        self._client.connect()
        input("We can see what objects the client has found. Press Enter to continue...")
        print(self._client.get_obj_names())
        self.arms = [self._client.get_obj_handle('raven_2/base_link_L'),
                     self._client.get_obj_handle('raven_2/base_link_R')]
        self.links = []
        for i in range(2):
            link_names = self.arms[i].get_children_names()
            for j in range(self.arms[i].get_num_of_children()):
                self.links.append(self._client.get_obj_handle('raven_2/' + link_names[j]))
        self.raven_type = ard.RAVEN_TYPE

        self.start_jp = np.zeros((2, 7))  # indexed at 0
        self.delta_jp = np.zeros((2, 7))
        self.home_joints = ard.HOME_JOINTS
        self.next_jp = np.zeros((2, 7))
        self.curr_tm = [0, 0]

        self.dance_scale_joints = ard.DANCE_SCALE_JOINTS
        self.loop_rate = ard.LOOP_RATE
        self.raven_joints = ard.RAVEN_JOINTS
        self.rc = [0, 0]
        self.rampup_count = np.array(self.rc)
        self.i = 0
        self.speed = 10.00 / self.loop_rate
        self.rampup_speed = 0.5 / self.loop_rate
        self.man_steps = 30

        self.homed = [False, False]
        self.moved = [False, False]
        self.finished = False
        self.limited = [False, False]

        print("\nHoming...\n")
        self.home_fast()
        self.set_curr_tm()
        print(self.curr_tm)

    def get_raven_type(self):
        return self.raven_type

    def set_curr_tm(self):
        for i in range(len(self.arms)):
            self.start_jp[i] = self.arms[i].get_all_joint_pos()
            self.curr_tm[i] = fk.fwd_kinematics(i, self.start_jp[i], ard)

    def home_fast(self):

        self.next_jp = [self.home_joints, self.home_joints]
        self.move()

        for j in range(len(self.moved)):
            self.homed[j] = self.moved[j]

        if all(self.homed):
            print("Raven is homed!")

        if not all(self.homed):
            print("Raven could not be homed, please try again :(")

    def sine_dance(self):
        if self.i == 0:
            # start = time.time()
            # similar to homing, moves raven incrementally in a sine pattern
            self.sine_dance_increment(1, 1, self.i, self.rampup_count)
            self.sine_dance_increment(1, 0, self.i, self.rampup_count)
        else:
            self.sine_dance_increment(0, 1, self.i, self.rampup_count)
            self.sine_dance_increment(0, 0, self.i, self.rampup_count)

        self.i += 1
        time.sleep(0.01)

    def sine_dance_increment(self, first_entry, arm, count, rampup_count):
        self.homed[arm] = False
        for i in range(self.raven_joints):
            offset = (i + arm) * math.pi / 2
            rampup = min(self.rampup_speed * self.rampup_count[arm], 1.0)
            self.arms[arm].set_joint_pos(i,
                                         rampup * self.dance_scale_joints[i] * math.sin(self.speed * (count + offset)) +
                                         self.home_joints[i])
            self.rampup_count[arm] += 1

    def get_t_command(self):
        return self.arms[0].get_torque_command(), self.arms[1].get_torque_command

    def get_raven_status(self):
        status = [(time.time())]

        # Add jpos for both arms
        for i in range(len(self.arms)):
            jpos = self.arms[i].get_all_joint_pos()
            jpos.insert(3, 0)
            status.extend(jpos)
            # status.extend(self.arms[i].get_all_joint_pos().insert(3, 0))  # 7 numbers

        # Placeholders for runlevel, sublevel, and last_seq
        for i in range(3):
            status.append(float("nan"))

        # Placeholders for type
        for i in range(2):
            status.append(float("nan"))

        # Placeholders for pos
        for i in range(6):
            status.append(float("nan"))

        # Placeholders for ori
        for i in range(18):
            status.append(float("nan"))

        # Placeholders for ori_d
        for i in range(18):
            status.append(float("nan"))

        # Placeholders for pos_d
        for i in range(6):
            status.append(float("nan"))

        # Placeholders for encVals
        for i in range(16):
            status.append(float("nan"))

        # Placeholders for dac_val
        for i in range(16):
            status.append(float("nan"))

        # Placeholders for Tau
        for i in range(16):
            status.append(float("nan"))

        # Placeholders for mpos
        for i in range(16):
            status.append(float("nan"))

        # Placeholders for mvel
        for i in range(16):
            status.append(float("nan"))

        # Add jvel for both arms
        for i in range(len(self.arms)):
            jvel = self.arms[i].get_all_joint_vel()
            jvel.insert(3, 0)
            status.extend(jvel)  # 7 numbers

        # Placeholders for jpos_d
        for i in range(16):
            status.append(float("nan"))

        # Placeholders for grasp_d
        for i in range(16):
            status.append(float("nan"))

        # Placeholders for encoffsets
        for i in range(16):
            status.append(float("nan"))

        # Placeholders for jac_vel
        for i in range(12):
            status.append(float("nan"))

        # Placeholders for jac_f
        for i in range(12):
            status.append(float("nan"))

        return status

    def set_raven_pos(self, pos_list):
        """
        sets raven position based on array containing positions from the physical
        raven robot. offsets are approximate and need finalization. indexing is intuitive
        """
        for i in range(len(pos_list)):
            if i == 0:
                self.arms[0].set_joint_pos(i, np.deg2rad(pos_list[i]) + (math.pi / 6))
            elif i == 1:
                self.arms[0].set_joint_pos(i, np.deg2rad(pos_list[i]) + (math.pi / 10))
            elif i == 2:
                self.arms[0].set_joint_pos(i, pos_list[i] / 100 - 0.26)
            elif i == 4:
                self.arms[0].set_joint_pos(i - 1, np.deg2rad(pos_list[i]) + (math.pi * 3) / 4)
            elif i == 5:
                self.arms[0].set_joint_pos(i - 1, np.deg2rad(pos_list[i]))
            elif i == 6:
                self.arms[0].set_joint_pos(i - 1, np.deg2rad(pos_list[i]) - math.pi / 12)
            elif i == 7:
                self.arms[0].set_joint_pos(i - 1, np.deg2rad(pos_list[i]) - math.pi / 12)
            elif i == 8:
                self.arms[1].set_joint_pos(i - 8, np.deg2rad(pos_list[i]) + math.pi / 6)
            elif i == 9:
                self.arms[1].set_joint_pos(i - 8, np.deg2rad(pos_list[i]) + math.pi / 10)
            elif i == 10:
                self.arms[1].set_joint_pos(i - 8, pos_list[i] / 100 - 0.26)
            elif i == 12:
                self.arms[1].set_joint_pos(i - 9, np.deg2rad(pos_list[i]) + (math.pi * 3) / 4)
            elif i == 13:
                self.arms[1].set_joint_pos(i - 9, np.deg2rad(pos_list[i]))
            elif i == 14:
                self.arms[1].set_joint_pos(i - 9, np.deg2rad(pos_list[i]) - math.pi / 12)
            elif i == 15:
                self.arms[1].set_joint_pos(i - 9, np.deg2rad(pos_list[i]) - math.pi / 12)

    def set_raven_force(self, pos_list):
        """
        a prototype for a similar method except using force instead of joint position
        """
        scale = 1
        for i in range(len(pos_list)):
            if i == 0:
                self.arms[0].set_joint_effort(i, (np.deg2rad(pos_list[i]) + (math.pi / 6)) / scale)
            elif i == 1:
                self.arms[0].set_joint_effort(i, (np.deg2rad(pos_list[i]) + (math.pi / 10)) / scale)
            elif i == 2:
                self.arms[0].set_joint_effort(i, (pos_list[i] / 100 - 0.26) / scale)
            elif i == 4:
                self.arms[0].set_joint_effort(i - 1, (np.deg2rad(pos_list[i]) + (math.pi * 3) / 4) / scale)
            elif i == 5:
                self.arms[0].set_joint_effort(i - 1, (np.deg2rad(pos_list[i])) / scale)
            elif i == 6:
                self.arms[0].set_joint_effort(i - 1, (np.deg2rad(pos_list[i]) - math.pi / 12) / scale)
            elif i == 7:
                self.arms[0].set_joint_effort(i - 1, (np.deg2rad(pos_list[i]) - math.pi / 12) / scale)
            elif i == 8:
                self.arms[1].set_joint_effort(i - 8, (np.deg2rad(pos_list[i]) + math.pi / 6) / scale)
            elif i == 9:
                self.arms[1].set_joint_effort(i - 8, (np.deg2rad(pos_list[i]) + math.pi / 10) / scale)
            elif i == 10:
                self.arms[1].set_joint_effort(i - 8, (pos_list[i] / 100 - 0.26) / scale)
            elif i == 12:
                self.arms[1].set_joint_effort(i - 9, (np.deg2rad(pos_list[i]) + (math.pi * 3) / 4) / scale)
            elif i == 13:
                self.arms[1].set_joint_effort(i - 9, (np.deg2rad(pos_list[i])) / scale)
            elif i == 14:
                self.arms[1].set_joint_effort(i - 9, (np.deg2rad(pos_list[i]) - math.pi / 12) / scale)
            elif i == 15:
                self.arms[1].set_joint_effort(i - 9, (np.deg2rad(pos_list[i]) - math.pi / 12) / scale)

    def plan_move(self, arm, delta_tm, gangle, p5=False, home_dh=ard.HOME_DH):
        """
        moves the desired robot arm based on inputted changes to cartesian coordinates
        Args:
            arm (int) : 0 for the left arm and 1 for the right arm
            delta_tm (numpy.array) : desired transformation matrix changes
            gangle (float) : the gripper angle, 0 is closed
            p5 (bool) : when false uses standard kinematics, when true uses p5 kinematics
            home_dh (array) : array containing home position, or desired postion of the
                joints not set by cartesian coordinates in inv_kinematics_p5
        """
        if arm == 1:
            gangle = -gangle

        self.start_jp[arm] = self.next_jp[arm]
        if p5:
            curr_tm = fk.fwd_kinematics_p5(arm, self.start_jp[arm], ard)
        else:
            curr_tm = fk.fwd_kinematics(arm, self.start_jp[arm], ard)
        curr_tm += delta_tm
        if p5:
            jpl = ik.inv_kinematics_p5(arm, curr_tm, gangle, home_dh, ard)
        else:
            jpl = ik.inv_kinematics(arm, curr_tm, gangle, ard)
        self.limited[arm] = jpl[1]
        # print("new jp: ", jpl)
        if self.limited[arm]:
            print("Desired cartesian position is out of bounds for Raven2. Will move to max pos.")
        new_jp = jpl[0]
        self.next_jp[arm] = new_jp

    def plan_move_abs(self, arm, tm, gangle, p5=False, home_dh=ard.HOME_DH):
        """
        Plans a move using the absolute cartesian position
        Args:
            arm (int) : 0 for the left arm and 1 for the right arm
            tm (numpy.array) : desired transformation matrix changes
            gangle (float) : the gripper angle, 0 is closed
            p5 (bool) : when false uses standard kinematics, when true uses p5 kinematics
            home_dh (array) : array containing home position, or desired postion of the
                joints not set by cartesian coordinates in inv_kinematics_p5
        """
        if arm == 1:
            gangle = -gangle

        self.start_jp[arm] = self.next_jp[arm]
        self.curr_tm[arm] += tm
        # print(self.curr_tm[arm])
        if p5:
            jpl = ik.inv_kinematics_p5(arm, self.curr_tm[arm], gangle, home_dh, ard)
        else:
            jpl = ik.inv_kinematics(arm, self.curr_tm[arm], gangle, ard)
        self.limited[arm] = jpl[1]
        if self.limited[arm]:
            print("Desired cartesian position is out of bounds for Raven2. Will move to max pos.")
        new_jp = jpl[0]
        self.next_jp[arm] = new_jp

    def calc_increment(self, arm):
        """
        Calculates the difference between the current joint positions and planned joint positions
        then calculates the number of increments required to stay within joint rotation limits
        Args:
            arm (int) : 0 for the left arm and 1 for the right arm
        """
        # Calculate delta jp
        # for i in range(self.raven_joints):
        #     self.start_jp[arm][i] = self.arms[arm].get_joint_pos(i)
        #     self.delta_jp[arm][i] = self.next_jp[arm][i] - self.arms[arm].get_joint_pos(i)

        self.delta_jp[arm] = self.next_jp[arm] - self.start_jp[arm]


        # Find safe increment
        increment = self.delta_jp[arm] / ard.MAX_JR
        # print("increments: ", increment)
        return max(map(abs, increment)) + 1

    def move_increment(self, arm, count, increments):
        """
        slowly increment the robot's position until it reaches the desired
        inputted position. uses a similar method structure to home
        """
        scale = min(1.0 * count / increments, 1.0)
        # array containing distance to go to start point
        diff_jp = [0, 0, 0, 0, 0, 0, 0]

        # sets position for each joint
        for i in range(self.arms[arm].get_num_joints()):
            self.arms[arm].set_joint_pos(i, scale * self.delta_jp[arm][i] + self.start_jp[arm][i])
            diff_jp[i] = abs(self.next_jp[arm][i] - self.arms[arm].get_joint_pos(i))

        # in progress, indicates when arm has reached next_jp
        max_value = np.max(diff_jp)

        if max_value < 0.1 or self.limited[arm]:
            self.moved[arm] = True

        else:
            self.moved[arm] = False
        return self.moved[arm]

    def move(self):
        """
        Uses the helper function move increment to move to the next planned position
        over a number of steps equal to man_steps or the number of increments required
        to keep each move within safe limits
        """
        # Find safe increment
        safe_increment = int(max(self.calc_increment(0), self.calc_increment(1)))

        if safe_increment <= self.man_steps:
            increments = self.man_steps
        else:
            increments = safe_increment

        # print("inc: ", increments)

        for i in range(increments):
            self.moved[0] = self.move_increment(0, i, increments)
            self.moved[1] = self.move_increment(1, i, increments)
            time.sleep(ard.COMMAND_TIME)

    def move_now(self, arm):
        """
        Moves robot to next jp
        Args:
            arm (int): 0 for the left arm and 1 for the right arm
        """
        # array containing the difference between next_jp and current joint position
        diff_jp = [0, 0, 0, 0, 0, 0, 0]

        for i in range(self.arms[arm].get_num_joints()):
            self.arms[arm].set_joint_pos(i, self.next_jp[arm][i])
            diff_jp[i] = abs(self.next_jp[arm][i] - self.arms[arm].get_joint_pos(i))

        max_value = np.max(diff_jp)

        if max_value < 0.1 or self.limited[arm]:
            self.moved[arm] = True

        else:
            self.moved[arm] = False
