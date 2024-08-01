import numpy as np
import math



def integral(t, x_1, x_2, size):
    """
    积分工具函数
    :param t: 采样时间
    :param x_1: 上一时刻的值
    :param x_2: 当前时刻的值
    :param size: 用于积分的数组大小
    :return: 与积分对象相同大小的数组
    """
    if size == 1:
        tm = t #采样时间
        x = (x_1 + x_2) / 2 #平均值
        y1 = x * tm
    elif size == 2:
        tm = np.array([t, t])
        x = np.add(x_1, x_2) / 2
        y1 = np.multiply(x, tm) #逐元素相乘
    elif size == 3:
        tm = np.array([t, t, t])
        x = np.add(x_1, x_2) / 2
        y1 = np.multiply(x, tm)
    elif size == 9:
        tm = np.array([t, t, t, t, t, t, t, t, t])
        x = np.add(x_1, x_2) / 2
        y1 = np.multiply(x, tm)
    return y1


def derivation(t, x_1, x_2, size):
    """
      导数工具函数
    :param t: 采样时间
    :param x_1: 上一时刻的值
    :param x_2: 当前时刻的值
    :param size: 用于求导的数组大小
    :return: 与导数对象相同大小的数组
    """
    if size == 1:
        tm = t
        x = x_2 - x_1
        y2 = x / tm
    elif size == 2:
        tm = np.array([t, t])
        x = np.array([x_2[0] - x_1[0], x_2[1] - x_1[1]])
        y2 = np.divide(x, tm)
    elif size == 3:
        tm = np.array([t, t, t])
        x = np.array([x_2[0] - x_1[0], x_2[1] - x_1[1], x_2[2] - x_1[2]])
        y2 = np.divide(x, tm)
    return y2


def sat_gd(u, a, size):
    """
  饱和函数
    :param u: 操作对象
    :param a: 饱和值
    :param size: 用于求导的数组大小。0：标量；其他：数组y
    """
    if size == 0:
        if u <= -a:
            return -a
        elif u >= a:
            return a
        else:
            return u
    else:
        u_max = 0
        for i in range(u.size):
            if abs(u[i]) > u_max:
                u_max = abs(u[i])
        if u_max <= a:
            return u
        else:
            return a * np.divide(u, u_max)


def vex(x):
    """
   获取一个3x3矩阵的反对称矩阵
    :param x: 一个3x3矩阵
    :return: 其反对称矩阵
    """
    return np.array([x[2][1], x[0][2], x[1][0]])


def normalizeWithGrad(x, xd):
    xSqrNorm = x[0] ** 2 + x[1] ** 2 + x[2] ** 2
    xNorm = np.sqrt(xSqrNorm)
    xNor = x / xNorm
    xNord = (xd - x * (np.dot(x, xd) / xSqrNorm)) / xNorm
    return xNor, xNord


def getR(attitude):
    """
      计算旋转矩阵
    :param attitude: 当前姿态
    :return: 当前旋转矩阵x
    """
    phi = attitude[0]
    theta = attitude[1]
    psi = attitude[2]
    sph = math.sin(phi)
    cph = math.cos(phi)
    st = math.sin(theta)
    ct = math.cos(theta)
    sps = math.sin(psi)
    cps = math.cos(psi)
    #计算R（5.9）
    R = np.array([ct * cps, cps * st * sph - sps * cph, cps * st * cph + sps * sph,
                  ct * sps, sps * st * sph + cps * cph, sps * st * cph - cps * sph,
                  -st, sph * ct, cph * ct])
    return R


def getApsi(psi):
    """
   计算4.2中的A_psi矩阵
    :param psi: 当前偏航角
    :return: 4.2中的A_psi矩阵
    """
    # 11.7
    sp = math.sin(psi)
    cp = math.cos(psi)
    R_psi = np.array([cp, -sp, sp, cp]).reshape(2, 2)
    x = np.array([0, 1, -1, 0]).reshape(2, 2)
    Apsi = np.dot(R_psi, x)
    return Apsi

def fhan(x1,x2,r,h):
    d = r*h*h
    a0 = h*x2
    y = x1+a0
    a1 =  math.sqrt(d*(d+8*abs(y)))
    a2 = a0 + np.sign(y)*(a1 - d)/2.0
    a =  (a0+y)*(np.sign(y+d)-np.sign(y-d))/2.0  + a2*(1-(np.sign(y+d)-np.sign(y-d))/2.0)
    fhan = -r*(a/d)*(np.sign(y+d)-np.sign(y-d))/2.0 - r*np.sign(a)*(1-(np.sign(a+d)-np.sign(a-d))/2.0)
    return fhan

def sat(x, delta):
    return  x/delta if np.abs(x)<delta else np.sign(x)

def fal(x, alpha, delta):
    return  x/np.power(delta,1-alpha) if np.abs(x)<delta else np.power(np.abs(x), alpha)*np.sign(x)



x1=0
x2=0
z1=0
z2=0
z3=0
u=0

def ADRC(v,y):
    """
      ADRC控制器
    :  v：输入的目标值；  
    ： y：反馈值
    :return: u
    """

# /****************ESO******************/
#     e  = 0,//误差
# 	  z1 = 0,//跟踪反馈值
# 	  z2 = 0,//跟踪反馈值的而微分
# 	  z3 = 0,//跟踪系统的扰动（总扰动）
# /**************NLSEF******************/
#     u = 0;//输出值
    
# float r = 0,//快速跟踪因子
#       h = 0;//滤波因子,系统调用步长

# /**************ESO**********/
# float b       = 0,//系统系数
#       delta   = 0,//delta为fal（e，alpha，delta）函数的线性区间宽度
#       belta01 = 0,//扩张状态观测器反馈增益1
# 	  belta02 = 0,//扩张状态观测器反馈增益2
# 	  belta03 = 0;//扩张状态观测器反馈增益3
	  
# /**************NLSEF*******/
# float alpha1 = 0,//
#       alpha2 = 0,//
#       belta1 = 0,//跟踪输入信号增益
#       belta2 = 0;//跟踪微分信号增益


# //参数区，这11个就是需要用户整定的参数
# /****************TD**********/
    r=40
    h=0.01
# /**************ESO**********/
    aa      = 0.0001
    b       = 0.000001
    delta   = 0.1
    belta01 = 3*aa
    belta02 = 3*aa*aa
    belta03 = aa*aa*aa        
	  
# /**************NLSEF*******/
    
    alpha1 = 0.4
    alpha2 = 1.6
    belta1 = 2
    belta2 = 5
                      
    #/******************************TD****************************************/
    global x2,x1,z1,z2,z3,u
    x1 =  h*x2 + x1
    x2 = x2 + h*fhan(x1-v,x2,r,h)
    #/******************************ESO***************************************/
    e = z1 - y
    z1 = z1 + h*(z2-belta01*e)
    z2 = z2 + h*(z3-belta02*fal(e,0.5,delta)+b*u)
    z3 = z3 + h*(-belta03*fal(e,0.25,delta))
    #/******************************NLSEF*************************************/
    e1 = x1 - z1
    e2 = x2 - z2
    
    u0 = belta1*fal(e1,alpha1,delta) + belta2*fal(e2,alpha2,delta)
    
    u = u0 - z3/b
    
    return u



x11=0
x12=0
z11=0
z12=0
z13=0
u1=0



def ADRC1(v,y):
    """
      ADRC控制器
    :  v：输入的目标值；  
    ： y：反馈值
    :return: u
    """

# /****************ESO******************/
#     e  = 0,//误差
# 	  z1 = 0,//跟踪反馈值
# 	  z2 = 0,//跟踪反馈值的而微分
# 	  z3 = 0,//跟踪系统的扰动（总扰动）
# /**************NLSEF******************/
#     u = 0;//输出值
    
# float r = 0,//快速跟踪因子
#       h = 0;//滤波因子,系统调用步长

# /**************ESO**********/
# float b       = 0,//系统系数
#       delta   = 0,//delta为fal（e，alpha，delta）函数的线性区间宽度
#       belta01 = 0,//扩张状态观测器反馈增益1
# 	  belta02 = 0,//扩张状态观测器反馈增益2
# 	  belta03 = 0;//扩张状态观测器反馈增益3
	  
# /**************NLSEF*******/
# float alpha1 = 0,//
#       alpha2 = 0,//
#       belta1 = 0,//跟踪输入信号增益
#       belta2 = 0;//跟踪微分信号增益


# //参数区，这11个就是需要用户整定的参数
# /****************TD**********/
    r=5
    h=0.01
# /**************ESO**********/
    aa      = 0.001
    b       = 0.001
    delta   = 0.1
    belta01 = 3*aa
    belta02 = 3*aa*aa
    belta03 = aa*aa*aa        
	  
# /**************NLSEF*******/
    
    alpha1 = 0.7
    alpha2 = 1.5
    belta1 = 5
    belta2 = 7
    #/******************************TD****************************************/
    global x12,x11,z11,z12,z13,u1
    x11 =  h*x12 + x11
    x12 = x12 + h*fhan(x11-v,x12,r,h)
    #/******************************ESO***************************************/
    e = z11 - y
    z11 = z11 + h*(z12-belta01*e)
    z12 = z12 + h*(z13-belta02*fal(e,0.5,delta)+b*u1)
    z13 = z13 + h*(-belta03*fal(e,0.25,delta))
    #/******************************NLSEF*************************************/
    e11 = x11 - z11
    e12 = x12 - z12
    
    u0 = belta1*fal(e11,alpha1,delta) + belta2*fal(e12,alpha2,delta)
    
    u1 = u0 - z13/b
    
    return u1

x21=0
x22=0
z21=0
z22=0
z23=0
u2=0



def ADRC2(v,y):
    """
      ADRC控制器
    :  v：输入的目标值；  
    ： y：反馈值
    :return: u
    """

# /****************ESO******************/
#     e  = 0,//误差
# 	  z1 = 0,//跟踪反馈值
# 	  z2 = 0,//跟踪反馈值的而微分
# 	  z3 = 0,//跟踪系统的扰动（总扰动）
# /**************NLSEF******************/
#     u = 0;//输出值
    
# float r = 0,//快速跟踪因子
#       h = 0;//滤波因子,系统调用步长

# /**************ESO**********/
# float b       = 0,//系统系数
#       delta   = 0,//delta为fal（e，alpha，delta）函数的线性区间宽度
#       belta01 = 0,//扩张状态观测器反馈增益1
# 	  belta02 = 0,//扩张状态观测器反馈增益2
# 	  belta03 = 0;//扩张状态观测器反馈增益3
	  
# /**************NLSEF*******/
# float alpha1 = 0,//
#       alpha2 = 0,//
#       belta1 = 0,//跟踪输入信号增益
#       belta2 = 0;//跟踪微分信号增益


# //参数区，这11个就是需要用户整定的参数
# /****************TD**********/
    r=5
    h=0.01
# /**************ESO**********/
    aa      = 0.004
    b       = 0.0002
    delta   = 0.1
    belta01 = 3*aa
    belta02 = 3*aa*aa
    belta03 = aa*aa*aa        
	  
# /**************NLSEF*******/
    
    alpha1 = 0.7
    alpha2 = 1.5
    belta1 = 7
    belta2 = 7
    #/******************************TD****************************************/
    global x22,x21,z21,z22,z23,u2
    x21 =  h*x22 + x21
    x22 = x22 + h*fhan(x21-v,x22,r,h)
    #/******************************ESO***************************************/
    e = z21 - y
    z21 = z21 + h*(z22-belta01*e)
    z22 = z22 + h*(z23-belta02*fal(e,0.5,delta)+b*u2)
    z23 = z23 + h*(-belta03*fal(e,0.25,delta))
    #/******************************NLSEF*************************************/
    e11 = x21 - z21
    e12 = x22 - z22
    
    u0 = belta1*fal(e11,alpha1,delta) + belta2*fal(e12,alpha2,delta)
    
    u2 = u0 - z23/b
    
    return u2
