本项目是基于python的无人机仿真实验，主要包含无人机运动学，动力学和控制分配模型，无人机3D飞行可视化等基础无人机仿真功能。利用针孔相机模型设计了虚拟相机，结合视觉伺服控制无人机对空中的移动目标实现追踪于反制。

1,无人机数学模型参考文件：/model/quadrotors_model.py
2,针孔相机模型参考文件：/model/image.py
3,控制器设计参考文件：/controllers/intercept.py
4,程序运行：main.py
5,无人机飞行可视化：/draw/draw3d.py/(update_plot)

视觉部分：
  相机模型主要利用了小孔成像的原理把目标点投影到成像平面上，针对相机平面对像素点的个数进行了限幅；
  由于相机的分辨率有限我们对成像的距离也进行了限制；

无人机控制部分：
  无人机的控制器采用经典的pid控制器；
  针对目标的追踪，我们采用了图像横轴误差作为偏航角的输入，使用pi控制实现机头始终朝向目标；
  对于横滚角采用期望值为0的单p控制；
  对俯仰角为跟踪图像的位置，以图像的纵轴误差作闭环，同时对俯仰角的期望进行了限幅使无人机保障向前运动；
  高度控制项目采用图像的纵轴误差乘以一个系数作为期望速度，结合速度闭环实现高度控制。

待续。。。
