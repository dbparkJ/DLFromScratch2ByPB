import math

# Conversion function from quaternions to Euler angles
def quaternion_to_euler(q):
    qw, qx, qy, qz = q
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    sin_pitch = 2 * (qw * qy - qz * qx)
    pitch = math.asin(sin_pitch) if abs(sin_pitch) < 1 else math.copysign(math.pi / 2, sin_pitch)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    return roll, pitch, yaw


def mahony_filter(angular_velocity, linear_acceleration, q,
                  dt = 0.01, integral_fb = [0.0, 0.0, 0.0], Kp = 2.0, Ki = 0.00):
    # global q, integral_fb

    # normalize
    norm_accel = math.sqrt(sum(a**2 for a in linear_acceleration))
    if norm_accel == 0:
        return  # 避免除以 0
    accel = [a / norm_accel for a in linear_acceleration]

    # Calculate the gravitational direction of the current quaternion
    vx = 2 * (q[1] * q[3] - q[0] * q[2])
    vy = 2 * (q[0] * q[1] + q[2] * q[3])
    vz = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2

    # Calculate attitude error (the difference between the estimated gravitational direction and the current gravitational direction)
    ex = accel[1] * vz - accel[2] * vy
    ey = accel[2] * vx - accel[0] * vz
    ez = accel[0] * vy - accel[1] * vx

    # Integral error correction
    integral_fb[0] += Ki * ex * dt
    integral_fb[1] += Ki * ey * dt
    integral_fb[2] += Ki * ez * dt

    # 修正后的角速度
    gx = angular_velocity[0] + Kp * ex + integral_fb[0]
    gy = angular_velocity[1] + Kp * ey + integral_fb[1]
    gz = angular_velocity[2] + Kp * ez + integral_fb[2]

    # Corrected angular velocity
    q_dot = [
        -0.5 * (q[1] * gx + q[2] * gy + q[3] * gz),
        0.5 * (q[0] * gx + q[2] * gz - q[3] * gy),
        0.5 * (q[0] * gy - q[1] * gz + q[3] * gx),
        0.5 * (q[0] * gz + q[1] * gy - q[2] * gx)
    ]

    # 更新四元数
    q[0] += q_dot[0] * dt
    q[1] += q_dot[1] * dt
    q[2] += q_dot[2] * dt
    q[3] += q_dot[3] * dt

    # normalize
    norm_q = math.sqrt(sum(qi**2 for qi in q))
    q = [qi / norm_q for qi in q]

    return quaternion_to_euler(q)

# # Mahony 滤波器参数
# Kp = 2.0  # 比例增益
# Ki = 0.005  # 积分增益
# integral_fb = [0.0, 0.0, 0.0]  # 积分误差
# # q = [1.0, 0.0, 0.0, 0.0]  # 初始四元数
# # # 示例数据（单位：rad/s、m/s^2、秒）
# # angular_velocity = [0.01, 0.02, 0.03]  # 陀螺仪数据
# # linear_acceleration = [0.0, 0.0, -9.81]  # 加速度计数据
# dt = 0.01  # 时间步长
#
# # 调用 Mahony 滤波器并输出姿态
# roll, pitch, yaw = mahony_filter(angular_velocity, linear_acceleration, q,
#                                  dt = 0.01, integral_fb = [0.0, 0.0, 0.0], Kp = 2.0, Ki = 0.005)
# print(f"Roll: {math.degrees(roll)}°, Pitch: {math.degrees(pitch)}°, Yaw: {math.degrees(yaw)}°")
