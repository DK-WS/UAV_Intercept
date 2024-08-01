import numpy as np
from tools import tools
from model import quadrotors_parameters as QPara
import math


class ControllerParameters:
    """
    This class sets the parameters of the controller and provides methods to reset the parameters
    """

    def __init__(self):
        self.tracking = 1
        self.position_des = np.array([1, 1, -10])  # [x,y,z]
        self.psi_des = 0 #目标偏航角
        # 位置PID参数
        self.K_ph_x = 0.5
        self.K_vh_x_p = 8
        self.K_vh_x_i = 0
        self.K_vh_x_d = 0.6
        self.K_ph_y = 1
        self.K_vh_y_p = 10
        self.K_vh_y_i = 0
        self.K_vh_y_d = 0.6
        self.K_pz = 1
        self.K_vz_p = 1
        self.K_vz_i = 0
        self.K_vz_d = 1
        # 姿态PID
        self.K_phi = 25
        self.K_wp_phi = 0.5
        self.K_wi_phi = 0
        self.K_wd_phi = 0.01

        self.K_theta = 25
        self.K_wp_theta = 0.5
        self.K_wi_theta = 0
        self.K_wd_theta = 0.01
        
        self.K_psi = 7
        self.K_wp_psi = 2
        self.K_wi_psi = 0
        self.K_wd_psi = 0
        # 参数矩阵
        self.K_ph = np.array([self.K_ph_x, 0, 0, self.K_ph_y]).reshape(2, 2)
        self.K_vh_p = np.array([self.K_vh_x_p, 0, 0, self.K_vh_y_p]).reshape(2, 2)
        self.K_vh_i = np.array([self.K_vh_x_i, 0, 0, self.K_vh_y_i]).reshape(2, 2)
        self.K_vh_d = np.array([self.K_vh_x_d, 0, 0, self.K_vh_y_d]).reshape(2, 2)
        self.K_attitude = np.array([self.K_phi, 0, 0,
                                    0, self.K_theta, 0,
                                    0, 0, self.K_psi]).reshape(3, 3)
        self.K_wp = np.array([self.K_wp_phi, 0, 0,
                              0, self.K_wp_theta, 0,
                              0, 0, self.K_wp_psi]).reshape(3, 3)
        self.K_wi = np.array([self.K_wi_phi, 0, 0,
                              0, self.K_wi_theta, 0,
                              0, 0, self.K_wi_psi]).reshape(3, 3)
        self.K_wd = np.array([self.K_wd_phi, 0, 0,
                              0, self.K_wd_theta, 0,
                              0, 0, self.K_wd_psi]).reshape(3, 3)

    def resetParameters(self):
        """
        重置参数
        """


class EulerController:
    """
    控制器类。
    提供自定义轨迹的接口
    """

    def __init__(self):
        self.ctrl_buffer = np.zeros(6)  # [position_des,attitude_des]
        # 用于存储四个电机的油门值，初始化为零数组
        self.throttle = np.zeros(4)
        # 用于存储误差的变量，分别表示水平速度误差的先前值和当前值、垂直速度误差的先前值和当前值以及姿态速度误差。
        self.err_vh_pre = np.zeros([2])
        self.err_vh_now = np.zeros([2])
        self.err_vz_pre = 0
        self.err_vz_now = 0
        self.err_attitude_v = np.zeros([3])
        # integration storage
        # 用于存储积分项的变量，分别表示水平速度误差的积分项、垂直速度误差的积分项和姿态速度误差的积分项。
        self.err_vh_i = 0
        self.err_vz_i = 0
        self.err_attitude_v_i = [0, 0, 0]

        self.tiao_pid = 0
        self.p_gain = 0.2
        self.adp_error = 0

    def eulerController(self, measurement, t, ctrl_para):
        """
        控制器的主要函数。
        :param measurement: 模型提供的当前测量值。[位置, 速度, 姿态, 姿态角速度]
        :param t: 当前时间
        :param ctrl_para: 控制器的参数，由ControllerParameters类提供
        """
        # position_des_now = np.zeros(3)  # 初始化 position_des_now
        if ctrl_para.tracking == 0:
            position_des_now = ctrl_para.position_des
        elif ctrl_para.tracking == 1:
            position_des_now = self.trajectory(t)

        if self.tiao_pid :
            trajectory_error = self.jisuan_error(measurement,position_des_now)
            ctrl_para.K_ph_x, ctrl_para.K_vh_x_p, ctrl_para.K_ph_y, ctrl_para.K_vh_y_p = self.tiaozheng_pid(
                ctrl_para.K_ph_x, ctrl_para.K_vh_x_p, ctrl_para.K_ph_y, ctrl_para.K_vh_y_p, trajectory_error)

            ctrl_para.K_ph = np.array([ctrl_para.K_ph_x, 0, 0, ctrl_para.K_ph_y]).reshape(2, 2)
            ctrl_para.K_vh_p = np.array([ctrl_para.K_vh_x_p, 0, 0, ctrl_para.K_vh_y_p]).reshape(2, 2)
            ctrl_para.K_vh_i = np.array([ctrl_para.K_vh_x_i, 0, 0, ctrl_para.K_vh_y_i]).reshape(2, 2)
            ctrl_para.K_vh_d = np.array([ctrl_para.K_vh_x_d, 0, 0, ctrl_para.K_vh_y_d]).reshape(2, 2)

        # measurement[位置012,速度345，姿态678，姿态角速度9 10 11]
            
        position_h_des_now = np.array([position_des_now[0], position_des_now[1]])
        A_psi = tools.getApsi(measurement[8])#用姿态计算Apsi
        position_h_now = measurement[0:2]#当前水平位置0 1
        # 水平位置通道模型（公式11.10-11.14）
        v_h_now = measurement[3:5]
        v_h_des_now = np.dot(ctrl_para.K_ph, position_h_des_now - position_h_now)

        self.err_vh_pre = self.err_vh_now
        self.err_vh_now = tools.sat_gd(v_h_now - v_h_des_now, 0.15, 1)

        self.err_vh_i = tools.integral(QPara.dt, self.err_vh_pre, self.err_vh_now, 2) + self.err_vh_i
        err_vh_d = tools.derivation(QPara.dt, self.err_vh_pre, self.err_vh_now, 2)

        p_out=np.dot(ctrl_para.K_vh_p, self.err_vh_now.reshape(2, 1)) 
        i_out=np.dot(ctrl_para.K_vh_i, self.err_vh_i.reshape(2, 1)) 
        d_out=np.dot(ctrl_para.K_vh_d,  err_vh_d.reshape(2, 1))
        pid_out= p_out +i_out + d_out
        ADRC1=-tools.ADRC1(v_h_des_now[0],v_h_now[0])
        ADRC2=-tools.ADRC2(v_h_des_now[1],v_h_now[1])
        attitude_h_des = np.dot(np.linalg.inv(A_psi),pid_out) / QPara.g

        AAA= np.array([ADRC1, ADRC2 ])

        # attitude_h_des = np.dot(np.linalg.inv(A_psi), AAA) / QPara.g
        attitude_h_des = tools.sat_gd(attitude_h_des, 0.15, 1).flatten()
        # 计算得到的水平姿态0 1 和期望的偏航角组成一个数组
        attitude_des_now = np.array([attitude_h_des[0], attitude_h_des[1], ctrl_para.psi_des])
        attitude_des_now = np.array([0, 0,0.785])
###############################################################################################################
        #高度
        position_z_now = measurement[2]
        velocity_z_des = -ctrl_para.K_pz * (position_z_now - position_des_now[2])
        self.err_vz_pre = self.err_vz_now
        self.err_vz_now = measurement[5] - velocity_z_des
        
        if 5 >= self.err_vz_now >= -5:
            self.err_vz_now = self.err_vz_now
        elif self.err_vz_now >= 5:
            self.err_vz_now = 5
        else:
            self.err_vz_now = -5

        self.err_vz_i = tools.integral(QPara.dt, self.err_vz_pre, self.err_vz_now, 1) + self.err_vz_i
        err_vz_d = tools.derivation(QPara.dt, self.err_vz_pre, self.err_vz_now, 1)
        #期望拉力
        f_des = QPara.m * (QPara.g + ctrl_para.K_vz_p * self.err_vz_now + ctrl_para.K_vz_i * self.err_vz_i
                            + ctrl_para.K_vz_d * err_vz_d)
        
        # f_des = QPara.m * (QPara.g - tools.ADRC(velocity_z_des, measurement[5]))
        # print("des:",AAA)
        if 25 >= f_des >= 0:
            f_des = f_des
        elif f_des > 25:
            f_des = 25
        else:
            f_des = 0
        ######################################################施工区#################################################################################
        #姿态控制
        err_attitude_now = tools.sat_gd(measurement[6:9] - attitude_des_now, 0.15, 1)
        attitude_v_des_now = -np.dot(ctrl_para.K_attitude, err_attitude_now.reshape(3, 1)).flatten()
        
        err_attitude_v_now = tools.sat_gd(measurement[9:] - attitude_v_des_now, 10, 1)
        self.err_attitude_v = np.vstack((self.err_attitude_v, err_attitude_v_now))

        self.err_attitude_v_i = tools.integral(QPara.dt, self.err_attitude_v[-2], self.err_attitude_v[-1],
                                               3) + self.err_attitude_v_i
        err_attitude_v_d_now = tools.derivation(QPara.dt, self.err_attitude_v[-2], self.err_attitude_v[-1], 3)

        tau_des = tools.sat_gd(-np.dot(ctrl_para.K_wp, self.err_attitude_v[-1].reshape(3, 1))
                               - np.dot(ctrl_para.K_wi, self.err_attitude_v_i.reshape(3, 1))
                               - np.dot(ctrl_para.K_wd, err_attitude_v_d_now.reshape(3, 1)), 10, 1).flatten()
        



        ########################################################施工区################################################################################
        # 期望拉力与期望力矩组成数组f_tau(6.24)
        f_tau = np.array([f_des, tau_des[0], tau_des[1], tau_des[2]])
        ang_v_motors_2 = np.dot(np.linalg.inv(QPara.thrust_matrix), f_tau).flatten()
        ang_v_motors = np.zeros([4])
        for i in range(4):
            if ang_v_motors_2[i] < 0:
                ang_v_motors[i] = 0
            else:
                ang_v_motors[i] = math.sqrt(ang_v_motors_2[i])
        self.throttle = ang_v_motors / QPara.Cr

        self.saveData(position_des_now, attitude_des_now)

    def trajectory(self, t):
        """
        Customize the track to be traced and get the desired position.
        :param t: Current time.
        :return: The desired position.
        """
        print("t:",t)
        # x_des_now = 0
        # y_des_now = 0
        # z_des_now = 1
        # if t<=3.14:
        #     x_des_now = 0
        #     y_des_now = 0
        #     z_des_now = 1
        # else:
        #     x_des_now = 5*np.cos(t/2)
        #     y_des_now = 5-5*np.sin(t/2)
        #     z_des_now = 5          
        
            

        # x_des_now = np.sin(t * np.pi / 10) + 2
        # y_des_now = np.cos(t * np.pi / 10) + 2
        # z_des_now = 10
        x_des_now = 0
        y_des_now = 10
        z_des_now = 0
        r=3
        position_des_now = np.array([x_des_now, y_des_now, z_des_now])
        # position_des_now = np.array([0, 0, 1])
        # position_des_now = (r * math.cos(2 * math.pi/20 * (t))-r, r * math.sin(2 * math.pi / 20*(t)),10)
        return position_des_now

    def jisuan_error(self,measurement,des_position):
        now_position = measurement[0:3]
        error = des_position - now_position
        return np.linalg.norm(error)

    def tiaozheng_pid(self,p1,p2,p3,p4,error):
        gain_error = error - self.adp_error
        p1 += gain_error * self.p_gain
        p2 += gain_error * self.p_gain
        p3 += gain_error * self.p_gain
        p4 += gain_error * self.p_gain
        self.adp_error = error
        return p1,p2,p3,p4


    def saveData(self, position_des_now, attitude_des_now):
        """
        Save the datas needed.
        :param position_des_now: The desired position.
        :param attitude_des_now: The desired attitude.
        """
        desired = np.concatenate((position_des_now, attitude_des_now))
        self.ctrl_buffer = np.vstack((self.ctrl_buffer, desired))  # save desired position and desired attitude
