
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# 生成一些示例数据
t = np.linspace(0, 10, 100)
x = np.sin(t)
y = np.cos(t)
z = t

# 创建图像和轴
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制动态过程
for i in range(len(t)):
    ax.clear()  # 清除之前的绘图
    ax.plot(x[:i], y[:i], z[:i])
    
    # 设置相同的缩放比例
    ax.set_box_aspect([np.ptp(x[:i]), np.ptp(y[:i]), np.ptp(z[:i])])

    # 设置标签和标题
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D动态过程')

    plt.pause(0.1)  # 暂停一段时间以便观察动画效果

plt.show()