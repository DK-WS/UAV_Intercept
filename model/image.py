import numpy as np

# 相机参数
focal_length = 0.02  # 焦距，单位为米（这里设为2厘米）
image_plane_width = 640  # 成像平面宽度，单位为米（36毫米）
image_plane_height = 480  # 成像平面高度，单位为米（24毫米）
u0=320
v0=240
fx=309.4362
fy=344.261
# 光心位置
optical_center = np.array([0, 0, 0.02])  # 光心在相机坐标系中的位置
image_plane_center = np.array([0, 0, 0])  # 成像平面中心在世界坐标系中的位置
Rcb=np.array([[0, 0, 1],[1,0,0],[0,1,0]])
Rbc=np.linalg.inv(Rcb)

import numpy as np

import numpy as np

def body_to_inertial_frame(body_point, roll, pitch, yaw):
    """
    Convert point from body frame (drone's coordinate system) to inertial frame.

    Args:
    - body_point: 3D vector representing coordinates in body frame (numpy array or list)
    - roll: Roll angle (in radians)
    - pitch: Pitch angle (in radians)
    - yaw: Yaw angle (in radians)
       
    Returns:
    - inertial_point: 3D vector representing coordinates in inertial frame (numpy array)
    """

    # Convert euler angles to rotation matrix (intrinsic rotations: yaw-pitch-roll)
    R_roll = np.array([[1, 0, 0],[0, np.cos(roll), np.sin(roll)],[0, -np.sin(roll), np.cos(roll)]])

    R_pitch = np.array([[np.cos(pitch), 0, -np.sin(pitch)],
                        [0, 1, 0],
                        [np.sin(pitch), 0, np.cos(pitch)]])

    R_yaw = np.array([[np.cos(-yaw), np.sin(-yaw), 0],
                      [-np.sin(-yaw), np.cos(-yaw), 0],
                      [0, 0, 1]])

    # RR=np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
    # Total rotation matrix from body to inertial frame (yaw-pitch-roll)
    R = np.dot(R_yaw, np.dot(R_pitch, R_roll))

    # Calculate inverse rotation matrix
    # R_inv = np.linalg.inv(R)

    # Convert body_point to numpy array if it's not already
    body_point = np.array(body_point)

    # Apply inverse rotation matrix to body_point
    # inertial_point =np.dot(RR,np.dot(R_inv, body_point))
    inertial_point =np.dot(R, body_point)


    return inertial_point



def inertial_to_body_frame(target_point, roll, pitch, yaw):
    """
    Convert target point from inertial frame to body frame (drone's coordinate system).

    Args:
    - target_point: 3D vector representing coordinates in inertial frame (numpy array or list)
    - roll: Roll angle (in radians)
    - pitch: Pitch angle (in radians)
    - yaw: Yaw angle (in radians)

    Returns:
    - body_point: 3D vector representing coordinates in body frame (numpy array)
    """

    # Convert euler angles to rotation matrix (intrinsic rotations: yaw-pitch-roll)
    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])
    # RR=np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
    # RR=np.array([[0, 1, 0],[-1, 0, 0],[0, 0, -1]])
    # Total rotation matrix from inertial to body frame (yaw-pitch-roll)
    R = np.dot(R_yaw, np.dot(R_pitch, R_roll))
    R_inv = np.linalg.inv(R)

    # Convert target_point to numpy array if it's not already
    target_point = np.array(target_point)

    # Apply rotation matrix to target_point
    # body_point = np.dot(RR,np.dot(R, target_point))
    body_point = np.dot(R_inv, target_point)

    print("body:",body_point)  # 打印当前模拟时间
    return body_point
    



def pinhole_camera_model(point_3d, roll, pitch, yaw):
    # 将三维点从世界坐标系转换到相机坐标系
    c_3d_body=inertial_to_body_frame(point_3d, roll, pitch, yaw)
    c_3d=np.dot(Rbc, c_3d_body)
    print("rr:",c_3d)

    if c_3d[2]<0.001 or c_3d[2]>20:
        u=0
        v=0
        flag=False
        print("Miss the target")
        return u, v,flag
    
    # u_pix=(fx*c_3d[1]+u0*c_3d[0])/c_3d[0]
    # v_pix=(fy*c_3d[2]+v0*c_3d[0])/c_3d[0]

    u_pix=(fx*c_3d[0]+u0*c_3d[2])/c_3d[2]
    v_pix=(fy*c_3d[1]+v0*c_3d[2])/c_3d[2]

    if u_pix<0.01 or u_pix>image_plane_width or v_pix<0.01 or v_pix>image_plane_height:
        u=0
        v=0
        flag=False
        print("Image out",u_pix,v_pix)
        return u, v,flag
    else:
        u=u_pix
        v=v_pix
        flag=True
        return u-320, v-240,flag 
    

        
        

    # point_in_camera_coords = c_3d - optical_center
    
    # # 计算相机坐标系中的图像坐标
    # image_x = focal_length * point_in_camera_coords[1] / point_in_camera_coords[0]
    # image_y = focal_length * point_in_camera_coords[2] / point_in_camera_coords[0]
    
    # # 将图像坐标调整为以成像平面中心为原点的坐标系
    # image_x += image_plane_center[0]
    # image_y += image_plane_center[1]
    
####示例使用

EXY=[]

# Example usage

roll = 0  # Example Roll angle
pitch = -0.785 # Example Pitch angle
yaw = 0  # Example Yaw angle




# point_3d_example = np.array([5, 5, 2])  # 示例三维点，单位为米
# image_x, image_y ,flag= pinhole_camera_model(point_3d_example, roll, pitch, yaw)
# print(f"三维点 {point_3d_example} 在成像平面上的图像坐标为：({image_x}, {image_y}) ")