import time
import math
import raven_ik as ik
import raven_fk as fk
import utilities as u
import numpy as np
import rospy
import physical_raven_def as prd
from physical_raven_arm import physical_raven_arm
import timeit


'''
author: Natalie Chalfant, Sean Fabrega
ambf_raven defines methods for an ambf_raven robot, including homnig, sine dance and hoping soon
cube tracing and soft body manipulation'''


class physical_raven:
    def __init__(self):

        rospy.init_node('raven_keyboard_controller', anonymous=True)

        self.arm_ctl_l = physical_raven_arm(name_space = ' ', robot_name = 'arm1', grasper_name = 'grasp1')
        self.arm_ctl_r = physical_raven_arm(name_space = ' ', robot_name = 'arm2', grasper_name = 'grasp2')
        self.arms = [self.arm_ctl_l, self.arm_ctl_r]
        self.raven_type = True

        self.start_jp = np.zeros((2, 7))  # indexed at 0
        self.delta_jp = np.zeros((2, 7))
        self.home_joints = prd.HOME_JOINTS
        self.next_jp = np.zeros((2, 7))
        self.jr = np.zeros((2, 7))
        self.curr_tm = [0, 0]

        self.dance_scale_joints = prd.DANCE_SCALE_JOINTS
        self.loop_rate = prd.LOOP_RATE
        self.raven_joints = prd.RAVEN_JOINTS
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

        # print("\nHoming...\n")
        # self.home_fast()
        self.set_curr_tm()
        print(self.curr_tm)
        print(self.start_jp)
        # self.resume()

    def get_raven_type(self):
        return self.raven_type

    def resume(self):
        self.arm_ctl_l.pub_state_command('resume')
        self.arm_ctl_r.pub_state_command('resume')

    def pause(self):
        self.arm_ctl_l.pub_state_command('pause')
        self.arm_ctl_r.pub_state_command('pause')

    def set_curr_tm(self):
        success = False
        while not success:
            time.sleep(1)
            for i in range(len(self.arms)):
                self.start_jp[i] = self.arms[i].get_measured_jpos()

            success = True

            for i in range(len(self.arms)):
                for j in range(len(self.start_jp[i])):
                    if math.isnan(self.start_jp[i, j]):
                        success = False
                        print("Unable to get Raven position, trying again...")

        for i in range(len(self.arms)):
            self.next_jp[i] = self.start_jp[i]
            self.curr_tm[i] = fk.fwd_kinematics(i, self.start_jp[i], prd)
            # add angles to change origin axes
            # self.curr_tm[i][0, 0] += math.cos(math.radians(60))
            # self.curr_tm[i][2, 2] += math.cos(math.radians(30))

    def home_fast(self):

        self.set_curr_tm()

        self.next_jp = [self.home_joints, self.home_joints]
        self.move()

        for j in range(len(self.moved)):
            self.homed[j] = self.moved[j]

        if all(self.homed):
            print("Raven is homed!")

        if not all(self.homed):
            print("Raven could not be homed, please try again :(")

    def sine_dance(self):
        # if self.i == 0:
        #     # start = time.time()
        #     # similar to homing, moves raven incrementally in a sine pattern
        #     self.sine_dance_increment(1, 1, self.i, self.rampup_count)
        #     self.sine_dance_increment(1, 0, self.i, self.rampup_count)
        # else:
        #     self.sine_dance_increment(0, 1, self.i, self.rampup_count)
        #     self.sine_dance_increment(0, 0, self.i, self.rampup_count)
        #
        # self.i += 1
        # time.sleep(0.01)
        print("not implemented :(")

    # def sine_dance_increment(self, first_entry, arm, count, rampup_count):
    #     self.homed[arm] = False
    #     for i in range(self.raven_joints):
    #         offset = (i + arm) * math.pi / 2
    #         rampup = min(self.rampup_speed * self.rampup_count[arm], 1.0)
    #         self.arms[arm].set_joint_pos(i,
    #                                      rampup * self.dance_scale_joints[i] * math.sin(self.speed * (count + offset)) +
    #                                      self.home_joints[i])
    #         self.rampup_count[arm] += 1

    def get_t_command(self):
        # return self.arms[0].get_torque_command(), self.arms[1].get_torque_command
        print("not implemented :(")

    def get_raven_status(self):
        msg = self.arms[0].get_raven_state()

        status = np.zeros((1, 240))
        timestr = "%.6f" % msg.hdr.stamp.to_sec()
        status[0] = timestr
        idx_count = 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.jpos[index])
            idx_count += 1

        status[0, idx_count] = ("%.6f" % msg.runlevel)
        idx_count += 1
        status[0, idx_count] = ("%.6f" % msg.sublevel)
        idx_count += 1
        status[0, idx_count] = ("%.6f" % msg.last_seq)
        idx_count += 1

        for index in range(0, 2):
            status[0, idx_count] = ("%.6f" % msg.type[index])
            idx_count += 1

        for index in range(0, 6):
            status[0, idx_count] = ("%.6f" % msg.pos[index])
            idx_count += 1

        for index in range(0, 18):
            status[0, idx_count] = ("%.6f" % msg.ori[index])
            idx_count += 1

        for index in range(0, 18):
            status[0, idx_count] = ("%.6f" % msg.ori_d[index])
            idx_count += 1

        for index in range(0, 6):
            status[0, idx_count] = ("%.6f" % msg.pos_d[index])
            idx_count += 1

        # newline[0, idx_count] = (msg.dt)
        # idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.encVals[index])
            idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.dac_val[index])
            idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.tau[index])
            idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.mpos[index])
            idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.mvel[index])
            idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.jvel[index])
            idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.mpos_d[index])
            idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.jpos_d[index])
            idx_count += 1

        for index in range(0, 2):
            status[0, idx_count] = ("%.6f" % msg.grasp_d[index])
            idx_count += 1

        for index in range(0, 16):
            status[0, idx_count] = ("%.6f" % msg.encoffsets[index])
            idx_count += 1

        for index in range(0, 12):
            status[0, idx_count] = ("%.6f" % msg.jac_vel[index])
            idx_count += 1

        for index in range(0, 12):
            status[0, idx_count] = ("%.6f" % msg.jac_f[index])
            idx_count += 1

        return status.tolist()[0]

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

    def plan_move(self, arm, tm, gangle, p5=False, home_dh=prd.HOME_DH):
        """
        moves the desired robot arm based on inputted changes to cartesian coordinates
        Args:
            arm (int) : 0 for the left arm and 1 for the right arm
            tm (numpy.array) : desired transformation matrix changes
            gangle (float) : the gripper angle, 0 is closed
            p5 (bool) : when false uses standard kinematics, when true uses p5 kinematics
            home_dh (array) : array containing home position, or desired postion of the
                joints not set by cartesian coordinates in inv_kinematics_p5
        """
        # curr_jp = np.array(self.arms[arm].get_all_joint_pos(), dtype="float")
        self.start_jp[arm] = self.next_jp[arm]
        if p5:
            curr_tm = fk.fwd_kinematics_p5(arm, self.start_jp[arm], prd)
        else:
            curr_tm = fk.fwd_kinematics(arm, self.start_jp[arm], prd)
        # print("initial tm :", curr_tm)
        # curr_tm[0, 3] += x
        # curr_tm[1, 3] += y
        # curr_tm[2, 3] += z
        curr_tm += tm
        if p5:
            jpl = ik.inv_kinematics_p5(arm, curr_tm, gangle, home_dh, prd)
        else:
            jpl = ik.inv_kinematics(arm, curr_tm, gangle, prd)
        self.limited[arm] = jpl[1]
        # print("new jp: ", jpl)
        if self.limited[arm]:
            print("Desired cartesian position is out of bounds for Raven2. Will move to max pos.")
        new_jp = jpl[0]
        print(new_jp)
        self.next_jp[arm] = new_jp

    def plan_move_abs(self, arm, tm, gangle, p5=False, home_dh=prd.HOME_DH):
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
        self.start_jp[arm] = self.next_jp[arm]
        tm[1, 3] *= -1
        self.curr_tm[arm] += tm
        # print("curr_tm: ", self.curr_tm[arm])
        if p5:
            jpl = ik.inv_kinematics_p5(arm, self.curr_tm[arm], gangle, home_dh, prd)
        else:
            jpl = ik.inv_kinematics(arm, self.curr_tm[arm], gangle, prd)
        self.limited[arm] = jpl[1]
        if self.limited[arm]:
            print("Desired cartesian position is out of bounds for Raven2. Will move to max pos.")
        new_jp = jpl[0]
        self.next_jp[arm] = new_jp
        # print("next_jp: ", self.next_jp)
        # print("start_jp: ", self.start_jp)

    def calc_increment(self, arm):
        """
        Calculates the difference between the current joint positions and planned joint positions
        then calculates the number of increments required to stay within joint rotation limits
        Args:
            arm (int) : 0 for the left arm and 1 for the right arm
        """

        self.delta_jp[arm] = self.next_jp[arm] - self.start_jp[arm]
        # print("arm", arm, " delta_jp: ", self.delta_jp[arm])

        # Find safe increment
        increment = self.delta_jp[arm] / prd.MAX_JR
        # print("increments: ", increment)
        return max(map(abs, increment)) + 1

    def move(self):
        # Find safe increment
        safe_increment = int(max(self.calc_increment(0), self.calc_increment(1)))
        # safe_increment = int(r2py_ctl_l.calc_increment())

        if safe_increment <= self.man_steps:
            increments = self.man_steps
        else:
            increments = safe_increment
        # print("inc:", increments)

        scale = 1 / increments
        for i in range(len(self.arms)):
            self.jr[i] = scale * self.delta_jp[i]
            # print(self.jr[i])

        for i in range(increments):
            self.arms[0].pub_jr_command(self.arms[0].seven2sixteen(self.jr[0]))
            self.arms[1].pub_jr_command(self.arms[1].seven2sixteen(self.jr[1]))
            # time.sleep(prd.COMMAND_TIME)

    # def move_now(self, arm):
    #     """
    #     Moves robot to next jp
    #     Args:
    #         arm (int): 0 for the left arm and 1 for the right arm
    #     """
    #     # array containing the difference between next_jp and current joint position
    #     diff_jp = [0, 0, 0, 0, 0, 0, 0]
    #
    #     for i in range(self.arms[arm].get_num_joints()):
    #         self.arms[arm].set_joint_pos(i, self.next_jp[arm][i])
    #         diff_jp[i] = abs(self.next_jp[arm][i] - self.arms[arm].get_joint_pos(i))
    #
    #     max_value = np.max(diff_jp)
    #
    #     if max_value < 0.1 or self.limited[arm]:
    #         self.moved[arm] = True
    #
    #     else:
    #         self.moved[arm] = False
