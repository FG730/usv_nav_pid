#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from scipy.signal import savgol_filter
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32

# 轨迹数据
global_planned_x, global_planned_y = [], []  # 存储完整的全局规划轨迹
all_planned_x, all_planned_y = [], []  # 累积局部规划轨迹
actual_x, actual_y = [], []  # 实际轨迹
left_thrust_data, right_thrust_data = [], []  # 存储左右推力数据
angular_error_data, turn_thrust_data = [], []  # 存储角度误差和转向推力数据
N = 5  # 每次局部规划路径只取前 N 个点

# 规划轨迹回调函数
def planned_path_callback(msg):
    global all_planned_x, all_planned_y, global_planned_x, global_planned_y

    # **全局规划轨迹（完整路径）**
    global_planned_x = [pose.pose.position.x for pose in msg.poses]
    global_planned_y = [pose.pose.position.y for pose in msg.poses]

    # **局部规划轨迹（仅取前 N 个点）**
    new_x = global_planned_x[:N]
    new_y = global_planned_y[:N]

    # **累积局部规划轨迹**
    all_planned_x.extend(new_x)
    all_planned_y.extend(new_y)

# 实际轨迹回调函数（保持完整路径）
def actual_path_callback(msg):
    actual_x.append(msg.pose.pose.position.x)
    actual_y.append(msg.pose.pose.position.y)

# 订阅左推力数据
def left_thrust_callback(msg):
    left_thrust_data.append(msg.data)

# 订阅右推力数据
def right_thrust_callback(msg):
    right_thrust_data.append(msg.data)

# 订阅角度误差数据
def angular_error_callback(msg):
    angular_error_data.append(msg.data)

# 订阅转向推力数据
def turn_thrust_callback(msg):
    turn_thrust_data.append(msg.data)

# 轨迹平滑处理（Savitzky-Golay 滤波）
def smooth_path(x, y, window_size=9, poly_order=3):
    if len(x) > window_size:
        try:
            if window_size % 2 == 0:
                window_size += 1  # 确保窗口大小为奇数
            x_smooth = np.array(x)
            y_smooth = savgol_filter(y, window_size, poly_order)
            return x_smooth, y_smooth
        except Exception as e:
            rospy.logwarn(f"Savitzky-Golay smoothing failed: {e}")
    return x, y  # 如果点数不足，返回原始路径

# Matplotlib 动画更新
def update_plot(frame):
    plt.clf()

    # **绘制轨迹图**
    plt.subplot(3, 1, 1)  # 第一行第一列
    plt.title("Planned vs. Actual Trajectory")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")

    if actual_y:
        ymin, ymax = min(actual_y), max(actual_y)
        y_range = ymax - ymin
        ymin, ymax = ymin - 0.2 * y_range, ymax + 0.2 * y_range
        plt.ylim(ymin, ymax)

        # **设置 y 轴刻度间隔为 10**
        y_ticks = np.arange(np.floor(ymin / 10) * 10, np.ceil(ymax / 10) * 10 + 10, 10)
        plt.yticks(y_ticks)

    # **绘制全局规划轨迹（绿色）**
    if global_planned_x and global_planned_y:
        plt.plot(global_planned_x, global_planned_y, 'g--', label="Global Planned Path", linewidth=1.5)

    # **绘制累积的局部规划轨迹（蓝色）**
    if all_planned_x and all_planned_y:
        smooth_x, smooth_y = smooth_path(all_planned_x, all_planned_y)
        plt.plot(smooth_x, smooth_y, 'c-', label="Smoothed Local Planned Path", linewidth=2)

    # **绘制实际轨迹（红色）**
    plt.plot(actual_x, actual_y, 'r-', label="Actual Path", linewidth=2)
    plt.legend()
    plt.grid(True)

    # **绘制角度误差和转向推力图**
    plt.subplot(3, 1, 2)  # 第二行第一列
    plt.title("Angular Error and Turn Thrust")
    plt.xlabel("Time")
    plt.ylabel("Value")

    if angular_error_data:
        plt.plot(angular_error_data, 'b-', label="Angular Error", linewidth=2)
    if turn_thrust_data:
        plt.plot(turn_thrust_data, 'm-', label="Turn Thrust", linewidth=2)
    plt.legend()
    plt.grid(True)

    # **绘制左右推力图**
    plt.subplot(3, 1, 3)  # 第三行第一列
    plt.title("Left and Right Thrust")
    plt.xlabel("Time")
    plt.ylabel("Thrust")

    if left_thrust_data:
        plt.plot(left_thrust_data, 'b-', label="Left Thrust", linewidth=2)
    if right_thrust_data:
        plt.plot(right_thrust_data, 'm-', label="Right Thrust", linewidth=2)
    plt.legend()
    plt.grid(True)

# 主函数
def main():
    rospy.init_node("trajectory_plotter", anonymous=True)

    # 订阅轨迹话题
    rospy.Subscriber("/move_base/TrajectoryPlannerROS/global_plan", Path, planned_path_callback)
    rospy.Subscriber("/cora/robot_localization/odometry/filtered", Odometry, actual_path_callback)

    # 订阅左右推力话题
    rospy.Subscriber("/cora/thrusters/left_thrust_cmd", Float32, left_thrust_callback)
    rospy.Subscriber("/cora/thrusters/right_thrust_cmd", Float32, right_thrust_callback)

    # 订阅角度误差和转向推力话题
    rospy.Subscriber("/angular_error", Float32, angular_error_callback)
    rospy.Subscriber("/turn_thrust", Float32, turn_thrust_callback)

    # Matplotlib 动画
    fig = plt.figure()
    ani = animation.FuncAnimation(fig, update_plot, interval=500)  # 500ms 更新一次
    plt.show()

if __name__ == "__main__":
    main()