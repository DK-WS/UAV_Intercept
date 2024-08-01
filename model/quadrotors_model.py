import numpy as np
from tools import tools as tools
from model import quadrotors_parameters as QPara
import math


class QuadrotorsModel:
    """
    这个类保存可能被使用的四轴飞行器的位置和姿态数据。
    方法 model 通过控制器给出的油门计算所需的数据。
    """

    def __init__(self):
        self.omega_motors = np.zeros([4])
        self.position_earth = np.zeros([3])
        self.velocity_earth = np.zeros([3])
        self.acceleration_earth = np.zeros([3])
        self.attitude = np.zeros([3])
        self.attitude_v = np.zeros([3])
        self.ang_velocity = np.zeros([3])
        self.ang_acceleration = np.zeros([3])
        self.measurement = np.zeros([12])  # 当前测量值。[位置,速度,姿态,姿态速度]
        self.measurement_buffer = np.zeros([12])  # [位置,速度,姿态,姿态速度]
        

    def step(self, throttle, t):
        """
        :param throttle: 四个电机的油门。
        :param t: 当前时间。
        """
        ang_v_motors = self.getAngVelMotors(throttle)

        f_tau = self.getTauAndF(ang_v_motors)

        f_now = f_tau[0]
        # print(f_now)
        tau_now = np.array(f_tau[1:])
        # print(tau_now)

        G_a = self.getGyroMoment(ang_v_motors)
        disturbance = self.disturbance()

        tau_now = tau_now.flatten() + disturbance[0:3]
        disturbance_f = np.array(disturbance[3:])

        # 计算机体角加速度
        ang_acceleration_now = (np.dot(np.linalg.inv(QPara.J), tau_now.reshape(3, 1) + G_a.reshape(3, 1)
                                       - np.cross(self.ang_velocity[-3:], np.dot(QPara.J, self.ang_velocity[-3:])
                                                  ).reshape(3, 1)).flatten())
        # 计算机体角速度
        ang_velocity_now = (tools.integral(QPara.dt, self.ang_acceleration[-3:], ang_acceleration_now,
                                           3) + self.ang_velocity[-3:])
        # 计算姿态速率
        W = getW(self.attitude[-3:])
        attitude_v_now = np.dot(W, ang_velocity_now.reshape(3, 1)).flatten()
        # 计算姿态
        attitude_now = (tools.integral(QPara.dt, self.attitude_v[-3:], attitude_v_now.reshape(1, 3), 3)
                        + self.attitude[-3:]).flatten()
        # 计算旋转矩阵
        rotation_matrix_now = tools.getR(self.attitude[-3:])
        # 计算加速度
        drag_now = -(QPara.C_drag * (self.velocity_earth[-3:]) ** 2).reshape(3, 1)
        acceleration_earth_now = (QPara.g * QPara.z_e + disturbance_f.reshape(3, 1)
                                  - f_now * np.dot(rotation_matrix_now.reshape(3, 3), QPara.z_e) / QPara.m + drag_now)
        # 计算速度
        velocity_earth_now = tools.integral(QPara.dt, self.acceleration_earth[-3:], acceleration_earth_now.flatten(),
                                            3) + self.velocity_earth[-3:]
        # 计算位置
        p_e_now = tools.integral(QPara.dt, self.velocity_earth[-3:], velocity_earth_now, 3) + self.position_earth[-3:]

        self.saveState(ang_acceleration_now.flatten(), ang_velocity_now.flatten(), attitude_v_now.flatten(),
                       attitude_now.flatten(), acceleration_earth_now.flatten(), velocity_earth_now.flatten(),
                       p_e_now.flatten())
        self.measure(p_e_now.flatten(), velocity_earth_now.flatten(), attitude_now.flatten(), attitude_v_now.flatten())

    def saveState(self, ang_acceleration_now, ang_velocity_now, attitude_v_now, attitude_now, acceleration_earth_now,
                  velocity_earth_now, p_e_now):
        """
        保存无人机的当前状态。
        函数入口参数是最新的四旋翼数据。
        """
        self.ang_acceleration = np.concatenate((self.ang_acceleration, ang_acceleration_now))
        self.ang_velocity = np.concatenate((self.ang_velocity, ang_velocity_now))
        self.attitude_v = np.concatenate((self.attitude_v, attitude_v_now))
        self.attitude = np.concatenate((self.attitude, attitude_now))
        self.acceleration_earth = np.concatenate((self.acceleration_earth, acceleration_earth_now))
        self.velocity_earth = np.concatenate((self.velocity_earth, velocity_earth_now))
        self.position_earth = np.concatenate((self.position_earth, p_e_now))

    def measure(self, p_e_now, velocity_earth_now, attitude_now, attitude_v_now):
        """
        用于控制飞行器的测量值。
        """
        noise = self.noise()
        self.measurement = np.concatenate((p_e_now + noise[0], velocity_earth_now + noise[1],
                                           attitude_now + noise[2], attitude_v_now + noise[3]))
        self.measurement_buffer = np.vstack((self.measurement_buffer, self.measurement))

    def getAngVelMotors(self, throttle):
        """
        计算电机的速度。
        :param throttle: 控制器的油门输出。
        :return: 四个电机的速度。
        """
        return QPara.Cr * throttle

    def getTauAndF(self, ang_v_motors):
        """
        计算四旋翼的当前推力和扭矩。
        :param ang_v_motors: 电机的速度。
        :return: [f,tau_x,tau_y,tau_z]。
        """
        ang_v_motors_2 = ang_v_motors ** 2
        [f, tau_x, tau_y, tau_z] = (np.dot(QPara.thrust_matrix, ang_v_motors_2.reshape(4, 1))).flatten()
        return [f, tau_x, tau_y, tau_z]

    def getGyroMoment(self, ang_v_motors):
        """
        计算四个电机产生的陀螺扭矩。
        :param ang_v_motors: 电机的速度。
        :return: 陀螺扭矩。
        """
        G_a_phi = (QPara.J_motor * self.ang_velocity[-2] * (ang_v_motors[0] - ang_v_motors[1]
                                                            + ang_v_motors[2] - ang_v_motors[3]))
        G_a_theta = (QPara.J_motor * self.ang_velocity[-3] * (-ang_v_motors[0] + ang_v_motors[1]
                                                              - ang_v_motors[2] + ang_v_motors[3]))
        G_a_psi = 0
        G_a = np.array([G_a_phi, G_a_theta, G_a_psi])
        return G_a

    def disturbance(self):
        """
        定制干扰。
        返回三轴扭矩和力。
        """
        [torque_x, torque_y, torque_z, f_x, f_y, f_z] = [0, 0, 0, 0, 0, 0]
        return [torque_x, torque_y, torque_z, f_x, f_y, f_z]

    def noise(self):
        """
        定制传感器测量过程中产生的噪声。
        """
        return np.array([[0, 0, 0],
                         [0, 0, 0],
                         [0, 0, 0],
                         [0, 0, 0]])  # [[位置],[速度],[姿态],[姿态速度]]


def getW(attitude):
    """
    :param attitude: 当前姿态。
    :return: 2.2中的 W 矩阵。
    """
    phi = attitude[0]
    theta = attitude[1]
    sph = math.sin(phi)
    cph = math.cos(phi)
    ct = math.cos(theta)
    tanthe = math.tan(theta)
    # zh 核对一下书本W表示，姿态变化率与机体角速度的关系
    W = np.array([1, tanthe * sph, tanthe * cph,
                  0, cph, -sph,
                  0, sph / ct, cph / ct]).reshape(3, 3)
    return W
