from model import quadrotors_parameters as QPara  # 导入quadrotors_parameters模块并命名为QPara
from model import quadrotors_model as QModel  # 导入quadrotors_model模块并命名为QModel
from controllers import euler_controller  # 导入euler_controller模块
from controllers import intercept  #导入拦截者

from draw import draw  # 导入draw模块
from draw import draw3d  # 导入draw3d模块
import time
# from draw import Animate
import matplotlib.pyplot as plt

def main():
    # euler_ctrl_para = euler_controller.ControllerParameters()  # 创建Euler控制器参数对象
    # euler_ctrl =euler_controller.EulerController()  # 创建Euler控制器对象
    # quadrotors_model = QModel.QuadrotorsModel()  # 创建四旋翼模型对象

    intercept_ctrl_para = intercept.Intercept_ControllerParameters()  # 创建Euler控制器参数对象
    intercept_ctrl =intercept.Intercept_Controller()  # 创建Euler控制器对象
    quadrotors_model = QModel.QuadrotorsModel()  # 创建四旋翼模型对象


    count=0
    t = [0]  # 创建时间列表，初始时间为0
    t_now = 0  # 当前时间初始化为0

    # while t_now < QPara.time:  # 当当前时间小于总时间时执行循环
    #     euler_ctrl.eulerController(quadrotors_model.measurement, t_now, euler_ctrl_para)  # 应用Euler控制器到四旋翼模型
    #     quadrotors_model.step(euler_ctrl.throttle, t_now)  # 更新四旋翼模型状态
    #     t_now += QPara.dt  # 增加当前时间步长
    #     count+=1
    #     if count % 5 == 0:
    #         draw3d.update_plot(euler_ctrl.ctrl_buffer,quadrotors_model.measurement_buffer)
    #     t.append(t_now)  # 将当前时间添加到时间列表中

        # print("sim:", t_now)  # 打印当前模拟时间
    #     # # print("K_ph_x:",euler_ctrl_para.K_ph_x)
    #     # # print("K_vh_x_p:",euler_ctrl_para.K_vh_x_p)
    #     # # print("K_ph_y:",euler_ctrl_para.K_ph_y)
    #     # # print("K_vh_y_p:",euler_ctrl_para.K_vh_y_p)
    #     # print("位置：",quadrotors_model.measurement)
    #     # print("油门", euler_ctrl.throttle)

    while t_now < QPara.time:  # 当当前时间小于总时间时执行循环
        intercept_ctrl.intercept_Controller(quadrotors_model.measurement, t_now, intercept_ctrl_para)  # 应用Euler控制器到四旋翼模型
        quadrotors_model.step(intercept_ctrl.throttle, t_now)  # 更新四旋翼模型状态
        t_now += QPara.dt  # 增加当前时间步长
        count+=1
        t.append(t_now)  # 将当前时间添加到时间列表中
        if count % 6 == 0:
            draw3d.update_plot(intercept_ctrl.ctrl_buffer,quadrotors_model.measurement_buffer)


        # time.sleep(QPara.dt)
        print("------------------------------------------------------------------------", t_now)  # 打印当前模拟时间


    print('finish simulation')  # 打印模拟结束信息


    # # #################绘制曲线
    # draw.draw(t, euler_ctrl.ctrl_buffer, quadrotors_model.measurement_buffer, 0, 0, "euler")
    # draw3d.draw3d(euler_ctrl.ctrl_buffer, quadrotors_model.measurement_buffer, 0, "euler3D")
    draw.draw(t, intercept_ctrl.ctrl_buffer, quadrotors_model.measurement_buffer, 0, 0, "intercept")
    draw3d.draw3d(intercept_ctrl.ctrl_buffer, quadrotors_model.measurement_buffer, 0, "intercept3D")
    plt.show()



if __name__ == '__main__':
    main()  # 调用main()函数进行程序执行