import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# 创建一个空的3D图像
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 定义物体的初始位置和速度
initial_position = (0, 0, 0)
velocity = (1, 1, 1)  # 每一步在x、y和z方向的移动距离

# 初始化轨迹数组
trajectory = np.array(initial_position)
position = np.array(initial_position)

# 更新函数，用于更新物体位置并绘制新的轨迹点
def update_trajectory(num):
    global position, trajectory
    position += velocity
    trajectory = np.vstack((trajectory, position))
    ax.clear()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], c='r', marker='o')  # 绘制轨迹点
    return ax



# # 创建动画
# ani = animation.FuncAnimation(fig, update_trajectory, frames=100, interval=50, blit=False)

# # 显示动画
# plt.show()
