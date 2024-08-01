import matplotlib.pyplot as plt


def draw(t, ctrl_buffer, measurement, eval, coordinate, fig_name):
    """
     绘制图形

    :param t: 时间序列
    :param ctrl_buffer: 包含期望位置和姿态
    :param measurement: 包含位置和姿态测量数据
    :param eval: 是否需要绘制评估指标；0: 是，其他: 否
    :param coordinate: 坐标系选择；0: Z轴向下，1: Z轴向上
    :param fig_name: 图形名称
    :return: 无返回值
    """
    position = measurement[:, 0:3]
    position_des = ctrl_buffer[:, 0:3]
    attitude = measurement[:, 6:9]
    attitude_des = ctrl_buffer[:, 3:]
    x = t
    # 绘制位置 x 和期望位置 x 的图
    plt.subplot(2, 3, 1)
    if coordinate == 0:
        y_1 = position[:, 0]
    elif coordinate == 1:
        y_1 = -position[:, 0]
    y_2 = position_des[:, 0]
    plt.plot(x, y_1, color='black', linewidth=1.2)
    plt.plot(x, y_2, '--', color='red', linewidth=1.2)
    plt.xlabel('time(s)', fontsize=10)
    plt.ylabel('Position_x(m)', fontsize=10)
    plt.legend(['x', 'position_x_des'], fontsize=8)
    plt.tick_params(labelsize=10)
    if eval == 0:
        evaluation(t, y_1, y_2)
    plt.xlim(xmin=0)
    # 绘制位置 y 和期望位置 y 的图
    plt.subplot(2, 3, 2)
    if coordinate == 0:
        y_1 = position[:, 1]
    elif coordinate == 1:
        y_1 = -position[:, 1]
    y_2 = position_des[:, 1]
    plt.plot(x, y_1, color='black', linewidth=1.2)
    plt.plot(x, y_2, '--', color='red', linewidth=1.2)
    plt.xlabel('time(s)', fontsize=10)
    plt.ylabel('Position_y(m)', fontsize=10)
    plt.legend(['y', 'position_y_des'], fontsize=8)
    plt.tick_params(labelsize=10)
    if eval == 0:
        evaluation(t, y_1, y_2)
    plt.xlim(xmin=0)
    # 绘制位置 z 和期望位置 z 的图
    plt.subplot(2, 3, 3)
    if coordinate == 0:
        y_1 = position[:, 2]
    elif coordinate == 1:
        y_1 = -position[:, 2]
    y_2 = position_des[:, 2]
    plt.plot(x, y_1, color='black', linewidth=1.2)
    plt.plot(x, y_2, '--', color='red', linewidth=1.2)
    plt.xlabel('time(s)', fontsize=10)
    plt.ylabel('Position_z(m)', fontsize=10)
    plt.legend(['z', 'position_z_des'], fontsize=8)
    plt.tick_params(labelsize=10)
    if eval == 0:
        evaluation(t, y_1, y_2)
    plt.xlim(xmin=0)
    # figure of phi and desired phi
    plt.subplot(2, 3, 4)
    y_1 = attitude[:, 0]
    y_2 = attitude_des[:, 0]
    plt.plot(x, y_1, color='black', linewidth=1.2)
    plt.plot(x, y_2, '--', color='red', linewidth=1.2)
    plt.xlabel('time(s)', fontsize=10)
    plt.ylabel('Attitude_phi(rad)', fontsize=10)
    plt.legend(['phi', 'phi_des'], fontsize=8)
    plt.tick_params(labelsize=10)
    plt.xlim(xmin=0)
    # figure of theta and desired theta
    plt.subplot(2, 3, 5)
    y_1 = attitude[:, 1]
    y_2 = attitude_des[:, 1]
    plt.plot(x, y_1, color='black', linewidth=1.2)
    plt.plot(x, y_2, '--', color='red', linewidth=1.2)
    plt.xlabel('time(s)', fontsize=10)
    plt.ylabel('Attitude_theta(rad)', fontsize=10)
    plt.legend(['theta', 'theta_des'], fontsize=8)
    plt.tick_params(labelsize=10)
    plt.xlim(xmin=0)
    # figure of psi and desired psi
    plt.subplot(2, 3, 6)
    y_1 = attitude[:, 2]
    y_2 = attitude_des[:, 2]
    plt.plot(x, y_1, color='black', linewidth=1.2)
    plt.plot(x, y_2, '--', color='red', linewidth=1.2)
    plt.xlabel('time(s)', fontsize=10)
    plt.ylabel('Attitude_psi(rad)', fontsize=10)
    plt.legend(['psi', 'psi_des'], fontsize=8)
    plt.tick_params(labelsize=10)
    plt.xlim(xmin=0)

    plt.tight_layout(pad=0.2, h_pad=0.2, w_pad=0.2)
    plt.subplots_adjust(left=0.103, bottom=0.105, right=0.977, top=0.969, wspace=0.8, hspace=0.277)
    plt.savefig('./images/{}.png'.format(fig_name), format='png', dpi=900, bbox_inches='tight')
    # plt.show()
    #plt.close()


def evaluation(t, y_1, y_2):
    """
     这是一个评估函数。
    通过上升时间和稳定时间来评估点控制效果

    :param t: 时间序列
    :param y_1: 测量数据
    :param y_2: 期望数据
    """
    x_tr = 0
    y_tr = 0
    x_ts = 0
    y_ts = 0
    for i in range(len(y_1)):
        if y_1[i] > y_2[-1] * 0.8:
            x_tr = t[i - 1]
            y_tr = y_1[i - 1]
            break

    for i in range(len(y_1)):
        if not y_2[-1] * 0.95 <= y_1[-i - 1] <= y_2[-1] * 1.05:
            x_ts = t[-i]
            y_ts = y_1[-i]
            break
    plt.scatter(x_tr, y_tr, color='blue', s=10)
    plt.text(1.2 * x_tr, y_tr, (round(x_tr, 2), round(y_tr, 2)))
    plt.scatter(x_ts, y_ts, color='blue', s=10)
    plt.text(1.1 * x_ts, 0.95 * y_ts, (round(x_ts, 2), round(y_ts, 2)))
