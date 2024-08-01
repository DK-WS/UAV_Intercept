import matplotlib.pyplot as plt
from model import image
import numpy as np
l=0.5
link1_body=[l,l,0]
link2_body=[-l,-l,0]
link3_body=[l,-l,0]
link4_body=[-l,l,0]

# def drawQuadrotor(measurement):
#     current_position = measurement[:, 0:3]
#     end_point1 = image.body_to_inertial_frame(link1_body,measurement[6], measurement[7], measurement[8]) + current_position[-1, :]
#     end_point2 = image.body_to_inertial_frame(link2_body,measurement[6], measurement[7], measurement[8]) + current_position[-1, :]
#     end_point3 = image.body_to_inertial_frame(link3_body,measurement[6], measurement[7], measurement[8]) + current_position[-1, :]
#     end_point4 = image.body_to_inertial_frame(link4_body,measurement[6], measurement[7], measurement[8]) + current_position[-1, :]
#     ax.plot([end_point1[0],current_position[-1, 0]], [end_point1[1],current_position[-1, 1]], [end_point1[2],current_position[-1, 2]], marker='o')
#     ax.plot([end_point2[0],current_position[-1, 0]], [end_point2[1],current_position[-1, 1]], [end_point2[2],current_position[-1, 2]], marker='o')
#     ax.plot([end_point3[0],current_position[-1, 0]], [end_point3[1],current_position[-1, 1]], [end_point3[2],current_position[-1, 2]], marker='o')
#     ax.plot([end_point4[0],current_position[-1, 0]], [end_point4[1],current_position[-1, 1]], [end_point4[2],current_position[-1, 2]], marker='o')







# fig = plt.figure()subplots()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d',aspect='equal')
# ax.set_xlim(ax.get_xlim()[::-1])

# # 反转 y 轴
# ax.set_ylim(ax.get_ylim()[::-1])

#     # 反转 z 轴
# ax.set_zlim(ax.get_zlim()[::-1])

def update_plot(ctrl_buffer,measurement):

    current_position = measurement[:, 0:3]
    attitude = measurement[:, 6:9]
    position_des = ctrl_buffer[:, 0:3]
    ax.clear()  # 清空之前的图形
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    
    # ax.set_xlim(0,5)  
    # ax.set_ylim(0,5) 
    # ax.set_zlim(-2,2) 

    end_point1 = image.body_to_inertial_frame(link1_body,attitude[-1, 0], attitude[-1,1], attitude[-1,2]) + current_position[-1, :]
    end_point2 = image.body_to_inertial_frame(link2_body,attitude[-1, 0], attitude[-1,1], attitude[-1,2]) + current_position[-1, :]
    end_point3 = image.body_to_inertial_frame(link3_body,attitude[-1, 0], attitude[-1,1], attitude[-1,2]) + current_position[-1, :]
    end_point4 = image.body_to_inertial_frame(link4_body,attitude[-1, 0], attitude[-1,1], attitude[-1,2]) + current_position[-1, :]

    ax.plot([end_point1[0],current_position[-1, 0]], [end_point1[1],current_position[-1, 1]], [end_point1[2],current_position[-1, 2]],color='blue')
    ax.plot([end_point2[0],current_position[-1, 0]], [end_point2[1],current_position[-1, 1]], [end_point2[2],current_position[-1, 2]],color='red')
    ax.plot([end_point3[0],current_position[-1, 0]], [end_point3[1],current_position[-1, 1]], [end_point3[2],current_position[-1, 2]],color='blue')
    ax.plot([end_point4[0],current_position[-1, 0]], [end_point4[1],current_position[-1, 1]], [end_point4[2],current_position[-1, 2]],color='red')
    # ax.set_box_aspect([np.ptp(current_position[:, 0]), np.ptp(current_position[:, 1]), np.ptp(current_position[:, 2])])
    # ax.set_box_aspect([2, 2, np.ptp(current_position[:, 2])])

    ax.scatter(position_des[:, 0], position_des[:, 1], position_des[:, 2], color='black',s=3)  # 绘制目标
    ax.scatter(current_position[:, 0], current_position[:, 1], current_position[:, 2], color='red',s=1)  # 绘制当前位置的点

    # ax.scatter(position_des[-1, 0], position_des[-1, 1], position_des[-1, 2], color='black',s=3)  # 绘制目标
    # ax.scatter(current_position[-1, 0], current_position[-1, 1], current_position[-1, 2], color='red',s=1)  # 绘制当前位置的点
    plt.pause(0.01)  # 稍作暂停，以便更新图形
    # plt.show()

def draw3d(ctrl_buffer, measurement, coordinate, fig_name):
    """
    绘制三维轨迹图。
    图的像将保存在名为images文件夹中。
    :param ctrl_buffer: 包括期望位置的数据。
    :param measurement: 包括位置测量数据。
    :param coordinate: 坐标系选择；0: Z轴向下，1: Z轴向上。
    :param fig_name: 图片名称。
    """
    position = measurement[:, 0:3]
    position_des = ctrl_buffer[:, 0:3]
    fig = plt.figure()
    #ax = fig.gca(projection='3d')
    ax = fig.add_subplot(111, projection='3d')
    # ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    # ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    # ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    if coordinate == 0:
        x = position[:, 0]
        y = position[:, 1]
        z = position[:, 2]
    elif coordinate == 1:
        x = -position[:, 0]
        y = -position[:, 1]
        z = -position[:, 2]
    ax.set_xlabel('x(m)', fontsize=10, labelpad=5)
    ax.set_ylabel('y(m)', fontsize=10, labelpad=5)
    ax.set_zlabel('z(m)', fontsize=10, labelpad=5)
    ax.plot3D(x, y, z, 'black')
    z_des = position_des[:, 2]
    x_des = position_des[:, 0]
    y_des = position_des[:, 1]
    ax.tick_params(labelsize=8)
    ax.plot3D(x_des, y_des, z_des, 'r:')
    ax.legend(['weizhi', 'qiwangweizhi'], fontsize=8, loc='upper left', bbox_to_anchor=(0, 0.8))
    ax.view_init(20, -30)
    print("test:",position[-1, :])
    plt.savefig('./images/{}.png'.format(fig_name), format='png', dpi=900, bbox_inches='tight')
    

