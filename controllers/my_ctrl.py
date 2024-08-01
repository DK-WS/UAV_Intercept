import numpy as np
from tools import  tools as tools
from model import quadrotors_parameters as qp
import math as math


class ctrl_para:#参数对象
    def __init__(self):
        self.psi_des = 0  # 目标偏航角
        self.position_des = np.array([1, 1, 10])
        #位置控制器，水平、高度
        #使用PID控制器
        self.ph_x_p = 1
        self.ph_x_i = 0
        self.ph_x_d = 0.0001
        
        self.ph_y_p= 1
        self.ph_y_i= 0
        self.ph_y_d = 0.0001

        self.pz_p = 2
        self.pz_i = 0
        self.pz_d = 0.01
        
        self.vz_p = 15
        self.vz_i = 0
        self.vz_d = 0.5


        #姿态控制器
        #滚转角
        self.phi_k = 25
        self.phi_p = 1
        self.phi_i = 0
        self.phi_d = 0
        #俯仰角
        self.theta_k = 25
        self.theta_p = 1
        self.theta_i = 0
        self.theta_d = 0
        #偏航角
        self.psi_k = 15
        self.psi_p = 1.5
        self.psi_i = 12
        self.psi_d = 0

        #参数矩阵，用于下方控制公式：
        self.K_php = np.array([self.ph_x_p,0,0,self.ph_y_p]).reshape(2,2)
        self.K_phd = np.array([self.ph_x_d,0,0,self.ph_y_d]).reshape(2,2)
        self.K_pzp = self.pz_p
        self.K_pzd = self.pz_d
        self.K_attitude = np.array([self.phi_k,0,0,
                              0,self.theta_k,0,
                              0,0,self.psi_k]).reshape(3,3)
        self.K_wp = np.array([self.phi_p,0,0,
                        0,self.theta_p,0,
                        0,0,self.psi_p]).reshape(3,3)
        self.K_wi = np.array([self.phi_i,0,0,
                        0,self.theta_i,0,
                        0,0,self.psi_i]).reshape(3,3)
        self.K_wd = np.array([self.phi_d, 0, 0,
                        0, self.theta_d, 0,
                        0, 0, self.psi_d]).reshape(3, 3)

class ctrl_tra:#控制器对象
    def __init__(self):
        self.ctrl_save = np.zeros(6)
        self.throttle = np.zeros(4)

        self.p_hd_pre = np.zeros([2])
        self.p_hd_now = np.zeros([2])
        self.p_hd_d_pre = np.zeros([2])
        self.p_hd_d_now = np.zeros([2])
        self.e_ph_now = np.zeros([2])
        self.e_ph_pre = np.zeros([2])

        self.p_zd_pre = 0
        self.p_zd_now = 0
        self.p_zd_d_pre = 0
        self.p_zd_d_now = 0
        self.e_pz_pre = 0
        self.e_pz_now = 0
###############################################################3
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
#############################################################33
        self.e_attitude_v = np.zeros([3])
        self.e_attitude_v_i = [0, 0, 0]


        self.psd = 1#定点/轨迹




    def ctrl_o(self,measurment,t,ctrl_pa):
        if self.psd == 1:
            self.p_d_now = self.trajectory(t)
        elif self.psd == 0:
            self.p_d_now = ctrl_pa.position_des

        self.p_h_pre = np.zeros(2)


        #水平位置通道
        self.p_hd_pre = self.p_hd_now
        self.p_hd_now =np.array([self.p_d_now[0],self.p_d_now[1]])
        A_psi = tools.getApsi(measurment[8])
        print(A_psi)
        self.p_h_now = measurment[0:2]
        p_h_d = tools.derivation(qp.dt,self.p_h_pre,self.p_h_now,2)
        self.e_ph_pre = self.e_ph_now
        self.e_ph_now = self.p_h_now - self.p_hd_now
        self.p_hd_d_pre = self.p_hd_d_now
        self.p_hd_d_now = tools.derivation(qp.dt,self.p_hd_pre,self.p_hd_now,2)
        # attitude_hd = (-np.dot(np.linalg.inv(A_psi),p_hd_dd.reshape(2,1)-np.dot(ctrl_pa.K_phd,e_ph_d.reshape(2,1))
        #                       -np.dot(ctrl_pa.K_php,self.e_ph_now.reshape(2,1)))/qp.g).flatten()

        attitude_hd = -np.dot(np.linalg.inv(A_psi),-np.dot(ctrl_pa.K_phd,p_h_d)
                              -np.dot(ctrl_pa.K_php,(self.p_h_now-self.p_hd_now)))/qp.g

        attitude_hd = tools.sat_gd(attitude_hd, 0.15, 1).flatten()
        attitude_hd_now = np.array([attitude_hd[0],attitude_hd[1],ctrl_pa.psi_des])


        #高度位置通道
        #p_z_pre = 0
        #self.p_zd_pre = self.p_zd_now
        p_z_now = measurment[2]
        velocity_z_des = -1* (p_z_now - self.p_d_now[2])
        self.err_vz_pre = self.err_vz_now
        self.err_vz_now = measurment[5] - velocity_z_des

        #p_z_d = tools.derivation(qp.dt,p_z_pre,p_z_now,1)
        #self.p_zd_now = self.p_d_now[2]
        
        if 5 >= self.err_vz_now >= -5:
            self.err_vz_now = self.err_vz_now
        elif self.err_vz_now >= 5:
            self.err_vz_now = 5
        else:
            self.err_vz_now = -5

        #self.p_zd_d_now = tools.derivation(qp.dt,self.p_zd_pre,self.p_zd_now,1)
        #self.p_zd_d_pre = self.p_zd_d_now

        self.err_vz_i = tools.integral(qp.dt, self.err_vz_pre, self.err_vz_now, 1) + self.err_vz_i
        err_vz_d = tools.derivation(qp.dt, self.err_vz_pre, self.err_vz_now, 1)

        # f_d = qp.m*qp.g - qp.m*(p_zd_dd-ctrl_pa.K_pzd*e_pz_d-ctrl_pa.K_pzp*self.e_pz_now)
        #f_d =qp.m*qp.g - qp.m*(-ctrl_pa.K_pzd*p_z_d-ctrl_pa.K_pzp*(p_z_now-self.p_zd_now))
        #f_d = tools.sat_gd(f_d,25,0)

        f_d = qp.m * (qp.g + ctrl_pa.K_pzp * self.err_vz_now + 0.1 * self.err_vz_i
                           + ctrl_pa.K_pzd * err_vz_d)
        
        #f_d = qp.m * (qp.g + tools.ADRC(velocity_z_des, measurment[5]))

        if 25 >= f_d >= 0:
            f_d = f_d
        elif f_d > 25:
            f_d= 25
        else:
            f_d = 0
########################################################################################################################################
        e_attitude_now =tools.sat_gd(measurment[6:9] - attitude_hd_now,10,1)
        attitude_v_des_now = -np.dot(ctrl_pa.K_attitude, e_attitude_now.reshape(3, 1)).flatten()
        phi_now=measurment[9]
        phi_d=attitude_v_des_now[0]

        e_attitude_v_now = measurment[9:] - attitude_v_des_now
        self.e_attitude_v = np.vstack((self.e_attitude_v, e_attitude_v_now))
        self.e_attitude_v_i = tools.integral(qp.dt, self.e_attitude_v[-2], self.e_attitude_v[-1],
                                               3) + self.e_attitude_v_i

        e_attitude_v_d_now = tools.derivation(qp.dt, self.e_attitude_v[-2], self.e_attitude_v[-1], 3)

        t_d =tools.sat_gd(-np.dot(ctrl_pa.K_wp, self.e_attitude_v[-1].reshape(3, 1))
                               - np.dot(ctrl_pa.K_wi, self.e_attitude_v_i.reshape(3, 1))
                               - np.dot(ctrl_pa.K_wd, e_attitude_v_d_now.reshape(3, 1)),10,1).flatten()
        #t_d[0]=tools.ADRC(phi_d,phi_now)
        #t_d[0]=0

##############################################################################################################################################
        f_tau = np.array([f_d, t_d[0], t_d[1], t_d[2]])
        ang_v_motors_2 = np.dot(np.linalg.inv(qp.thrust_matrix), f_tau).flatten()
        ang_v_motors = np.zeros([4])
        for i in range(4):
            if ang_v_motors_2[i] < 0:
                ang_v_motors[i] = 0
            # elif ang_v_motors_2[i]>=5:
            #     ang_v_motors[i] = 5
            else:
                ang_v_motors[i] = math.sqrt(ang_v_motors_2[i])

        self.throttle = ang_v_motors / qp.Cr

        self.saveData(self.p_d_now, attitude_hd_now)





    def trajectory(self, t):


        # #sin和cos的轨迹
        # x_des_now = np.sin(t * np.pi / 10) + 2
        # y_des_now = np.cos(t * np.pi / 10) + 2
        # z_des_now = 10


        # #直线*(PID参数同COS SIN)
        x_des_now = t
        y_des_now = t
        z_des_now = 10



        position_des_now = np.array([x_des_now, y_des_now, z_des_now])
        return position_des_now

    def saveData(self, position_des_now, attitude_des_now):

        desired = np.concatenate((position_des_now, attitude_des_now))
        self.ctrl_save = np.vstack((self.ctrl_save, desired))
