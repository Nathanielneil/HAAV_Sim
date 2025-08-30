#!/usr/bin/env python3
"""
完整ROV演示程序
展示所有核心功能：推进器控制、轨迹跟踪、传感器读取、水动力学仿真
"""

import sys
import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread, Event
import json

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'python'))

from rov_client import ROVController
from control_algorithms import ROVControlSystem, Waypoint, ControlMode

class ROVDemo:
 """完整ROV演示类"""
 
 def __init__(self):
 self.rov = None
 self.control_system = ROVControlSystem()
 self.demo_running = False
 self.data_logger = []
 
 # 演示参数
 self.demo_scenarios = {
 'basic_movement': self.demo_basic_movement,
 'trajectory_following': self.demo_trajectory_following,
 'sensor_collection': self.demo_sensor_collection,
 'station_keeping': self.demo_station_keeping,
 'obstacle_avoidance': self.demo_obstacle_avoidance,
 'complete_mission': self.demo_complete_mission
 }
 
 def initialize_rov(self) -> bool:
 """初始化ROV系统"""
 try:
 print(" 初始化ROV系统...")
 self.rov = ROVController()
 
 if self.rov.initialize():
 print(" ROV初始化成功")
 
 # 显示初始状态
 state = self.rov.get_rov_state()
 if state:
 print(f" 初始位置: ({state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f})")
 print(f" 当前深度: {state.depth:.2f}m")
 
 return True
 else:
 print(" ROV初始化失败")
 return False
 
 except Exception as e:
 print(f" ROV初始化异常: {e}")
 return False
 
 def demo_basic_movement(self):
 """基础移动演示"""
 print("\n 演示1: 基础移动控制")
 print("=" * 40)
 
 try:
 # 获取初始位置
 initial_state = self.rov.get_rov_state()
 print(f" 起始位置: ({initial_state.position[0]:.2f}, {initial_state.position[1]:.2f}, {initial_state.position[2]:.2f})")
 
 # 演示不同方向的移动
 movements = [
 ("前进", [0.5, 0.5, 0.5, 0.5, 0.6, 0.4, 0.6, 0.4]), # 前进
 ("左移", [0.5, 0.5, 0.5, 0.5, 0.4, 0.6, 0.4, 0.6]), # 左移
 ("上浮", [0.6, 0.6, 0.6, 0.6, 0.5, 0.5, 0.5, 0.5]), # 上浮
 ("右转", [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.6, 0.4]), # 右转
 ]
 
 for direction, pwm_values in movements:
 print(f"\n {direction}移动 (3秒)...")
 
 self.rov.set_thruster_pwm(pwm_values, 3.0)
 
 # 监控移动过程
 for i in range(6):
 time.sleep(0.5)
 state = self.rov.get_rov_state()
 if state:
 print(f" {i*0.5:.1f}s: 位置=({state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f})")
 
 # 停止推进器
 self.rov.emergency_stop()
 time.sleep(1)
 
 # 显示最终位置
 final_state = self.rov.get_rov_state()
 total_distance = np.linalg.norm(final_state.position - initial_state.position)
 print(f"\n 总移动距离: {total_distance:.2f}m")
 print(" 基础移动演示完成")
 
 except Exception as e:
 print(f" 基础移动演示失败: {e}")
 
 def demo_trajectory_following(self):
 """轨迹跟踪演示"""
 print("\n 演示2: 轨迹跟踪控制")
 print("=" * 40)
 
 try:
 # 设置控制模式
 self.control_system.set_control_mode(ControlMode.TRAJECTORY)
 
 # 定义方形轨迹
 square_waypoints = [
 Waypoint(0, 0, -2, 0),
 Waypoint(3, 0, -2, 0),
 Waypoint(3, 3, -2, math.pi/2),
 Waypoint(0, 3, -2, math.pi),
 Waypoint(0, 0, -2, -math.pi/2)
 ]
 
 print(" 规划方形轨迹...")
 if self.control_system.plan_mission(square_waypoints):
 print(" 轨迹规划完成")
 
 # 执行轨迹跟踪
 print(" 开始轨迹跟踪...")
 
 start_time = time.time()
 trajectory_data = []
 
 while time.time() - start_time < 60: # 最多运行60秒
 current_state = self.rov.get_rov_state()
 if not current_state:
 continue
 
 # 准备状态字典
 state_dict = {
 'x': current_state.position[0],
 'y': current_state.position[1],
 'z': current_state.position[2],
 'yaw': current_state.orientation[2]
 }
 
 # 计算控制输出
 control_output = self.control_system.execute_control(state_dict, {})
 
 # 转换为推进器控制
 pwm_values = self.rov.wrench_to_pwm(control_output)
 self.rov.set_thruster_pwm(pwm_values.tolist(), 0.1)
 
 # 记录数据
 trajectory_data.append({
 'time': time.time() - start_time,
 'position': current_state.position.copy(),
 'control': control_output.copy()
 })
 
 # 每5秒显示一次状态
 if len(trajectory_data) % 50 == 0:
 elapsed = time.time() - start_time
 print(f" {elapsed:.1f}s: 位置=({current_state.position[0]:.2f}, {current_state.position[1]:.2f}, {current_state.position[2]:.2f})")
 
 time.sleep(0.1)
 
 self.rov.emergency_stop()
 print(" 轨迹跟踪演示完成")
 
 # 保存数据用于分析
 self.save_trajectory_data(trajectory_data, "square_trajectory")
 
 else:
 print(" 轨迹规划失败")
 
 except Exception as e:
 print(f" 轨迹跟踪演示失败: {e}")
 
 def demo_sensor_collection(self):
 """传感器数据采集演示"""
 print("\n 演示3: 传感器数据采集")
 print("=" * 40)
 
 try:
 # 采集不同传感器数据
 print(" 采集相机图像...")
 rgb_image = self.rov.get_camera_image("front_center")
 if rgb_image is not None:
 print(f" RGB图像: {rgb_image.shape}")
 # 保存图像
 plt.imsave(f"demo_rgb_{int(time.time())}.png", rgb_image)
 
 print(" 采集深度图像...")
 depth_image = self.rov.get_depth_image()
 if depth_image is not None:
 print(f" 深度图像: {depth_image.shape}")
 # 保存深度图
 plt.imsave(f"demo_depth_{int(time.time())}.png", depth_image, cmap='plasma')
 
 print(" 采集IMU数据...")
 imu_data = self.rov.get_imu_data()
 if imu_data:
 print(" IMU数据采集成功:")
 print(f" 加速度: {imu_data['acceleration']}")
 print(f" 角速度: {imu_data['angular_velocity']}")
 print(f" 姿态: {imu_data['orientation']}")
 
 # 连续采集一段时间的传感器数据
 print(" 连续传感器数据采集 (10秒)...")
 sensor_data_log = []
 
 start_time = time.time()
 while time.time() - start_time < 10:
 rov_state = self.rov.get_rov_state()
 imu_data = self.rov.get_imu_data()
 
 if rov_state and imu_data:
 data_point = {
 'timestamp': time.time() - start_time,
 'position': rov_state.position.tolist(),
 'velocity': rov_state.velocity.tolist(),
 'orientation': rov_state.orientation.tolist(),
 'depth': rov_state.depth,
 'acceleration': imu_data['acceleration'],
 'angular_velocity': imu_data['angular_velocity']
 }
 sensor_data_log.append(data_point)
 
 time.sleep(0.1)
 
 print(f" 采集了{len(sensor_data_log)}个数据点")
 
 # 保存传感器数据
 self.save_sensor_data(sensor_data_log, "sensor_demo")
 
 # 数据统计
 if sensor_data_log:
 positions = np.array([d['position'] for d in sensor_data_log])
 velocities = np.array([d['velocity'] for d in sensor_data_log])
 
 print(" 数据统计:")
 print(f" 位置范围: X[{positions[:, 0].min():.2f}, {positions[:, 0].max():.2f}]")
 print(f" Y[{positions[:, 1].min():.2f}, {positions[:, 1].max():.2f}]")
 print(f" Z[{positions[:, 2].min():.2f}, {positions[:, 2].max():.2f}]")
 print(f" 最大速度: {np.linalg.norm(velocities, axis=1).max():.2f} m/s")
 
 print(" 传感器采集演示完成")
 
 except Exception as e:
 print(f" 传感器采集演示失败: {e}")
 
 def demo_station_keeping(self):
 """定点保持演示"""
 print("\n 演示4: 定点保持控制")
 print("=" * 40)
 
 try:
 # 获取当前位置作为基准
 current_state = self.rov.get_rov_state()
 target_position = current_state.position + np.array([2, 1, -1]) # 目标位置
 target_heading = math.pi / 4 # 45度航向
 
 print(f" 目标位置: ({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f})")
 print(f" 目标航向: {math.degrees(target_heading):.1f}°")
 
 # 设置控制模式
 self.control_system.set_control_mode(ControlMode.STATION_KEEPING)
 
 # 定点保持控制
 start_time = time.time()
 position_errors = []
 heading_errors = []
 
 print(" 开始定点保持...")
 
 while time.time() - start_time < 30: # 保持30秒
 current_state = self.rov.get_rov_state()
 if not current_state:
 continue
 
 # 准备状态和目标
 state_dict = {
 'x': current_state.position[0],
 'y': current_state.position[1],
 'z': current_state.position[2],
 'yaw': current_state.orientation[2]
 }
 
 target_dict = {
 'x': target_position[0],
 'y': target_position[1],
 'z': target_position[2],
 'yaw': target_heading
 }
 
 # 控制计算
 control_output = self.control_system.execute_control(state_dict, target_dict)
 
 # 应用控制
 pwm_values = self.rov.wrench_to_pwm(control_output)
 self.rov.set_thruster_pwm(pwm_values.tolist(), 0.1)
 
 # 记录误差
 pos_error = np.linalg.norm(current_state.position - target_position)
 heading_error = abs(current_state.orientation[2] - target_heading)
 
 position_errors.append(pos_error)
 heading_errors.append(heading_error)
 
 # 每5秒显示状态
 if len(position_errors) % 50 == 0:
 elapsed = time.time() - start_time
 print(f" {elapsed:.1f}s: 位置误差={pos_error:.3f}m, 航向误差={math.degrees(heading_error):.1f}°")
 
 time.sleep(0.1)
 
 self.rov.emergency_stop()
 
 # 性能统计
 avg_pos_error = np.mean(position_errors)
 avg_heading_error = np.mean(heading_errors)
 
 print("\n 定点保持性能:")
 print(f" 平均位置误差: {avg_pos_error:.3f}m")
 print(f" 平均航向误差: {math.degrees(avg_heading_error):.1f}°")
 print(f" 最终位置误差: {position_errors[-1]:.3f}m")
 
 print(" 定点保持演示完成")
 
 except Exception as e:
 print(f" 定点保持演示失败: {e}")
 
 def demo_obstacle_avoidance(self):
 """障碍物避让演示"""
 print("\n 演示5: 障碍物避让")
 print("=" * 40)
 
 try:
 # 添加虚拟障碍物
 self.control_system.obstacle_avoidance.clear_obstacles()
 
 # 在路径上放置障碍物
 obstacles = [
 (2, 1, -2, 1.0), # (x, y, z, radius)
 (4, 3, -1.5, 0.8),
 (1, 4, -2.5, 1.2)
 ]
 
 for obs_x, obs_y, obs_z, radius in obstacles:
 self.control_system.obstacle_avoidance.add_obstacle(
 np.array([obs_x, obs_y, obs_z]), radius
 )
 print(f" 添加障碍物: ({obs_x}, {obs_y}, {obs_z}), 半径={radius}m")
 
 # 设置起点和终点
 current_state = self.rov.get_rov_state()
 start_pos = current_state.position
 goal_pos = start_pos + np.array([5, 5, -2])
 
 print(f" 起点: ({start_pos[0]:.1f}, {start_pos[1]:.1f}, {start_pos[2]:.1f})")
 print(f" 终点: ({goal_pos[0]:.1f}, {goal_pos[1]:.1f}, {goal_pos[2]:.1f})")
 
 # 规划避障路径
 print(" 规划避障路径...")
 avoiding_path = self.control_system.obstacle_avoidance.plan_avoiding_path(
 start_pos, goal_pos, num_iterations=150
 )
 
 if len(avoiding_path) > 1:
 print(f" 规划成功，路径长度: {len(avoiding_path)}点")
 
 # 设置路径跟踪
 self.control_system.path_follower.set_path(avoiding_path)
 self.control_system.set_control_mode(ControlMode.TRAJECTORY)
 
 # 执行避障导航
 print(" 开始避障导航...")
 
 start_time = time.time()
 path_index = 0
 
 while time.time() - start_time < 60 and path_index < len(avoiding_path) - 1:
 current_state = self.rov.get_rov_state()
 if not current_state:
 continue
 
 # 检查是否接近当前目标点
 current_target = avoiding_path[path_index]
 distance_to_target = np.linalg.norm(current_state.position - current_target)
 
 if distance_to_target < 0.5: # 接近目标，移动到下一点
 path_index = min(path_index + 1, len(avoiding_path) - 1)
 print(f" 到达路径点 {path_index}/{len(avoiding_path)}")
 
 # 计算控制输出
 target_dict = {
 'x': avoiding_path[path_index][0],
 'y': avoiding_path[path_index][1],
 'z': avoiding_path[path_index][2],
 'yaw': 0.0
 }
 
 state_dict = {
 'x': current_state.position[0],
 'y': current_state.position[1],
 'z': current_state.position[2],
 'yaw': current_state.orientation[2]
 }
 
 control_output = self.control_system.execute_control(state_dict, target_dict)
 
 # 应用控制
 pwm_values = self.rov.wrench_to_pwm(control_output)
 self.rov.set_thruster_pwm(pwm_values.tolist(), 0.1)
 
 time.sleep(0.1)
 
 # 检查是否到达目标
 final_state = self.rov.get_rov_state()
 final_distance = np.linalg.norm(final_state.position - goal_pos)
 
 print(f"\n 避障导航结果:")
 print(f" 最终距离目标: {final_distance:.2f}m")
 print(f" 路径完成度: {path_index}/{len(avoiding_path)}")
 
 if final_distance < 1.0:
 print(" 成功到达目标区域")
 else:
 print(" 未完全到达目标")
 
 else:
 print(" 路径规划失败")
 
 self.rov.emergency_stop()
 print(" 障碍物避让演示完成")
 
 except Exception as e:
 print(f" 障碍物避让演示失败: {e}")
 
 def demo_complete_mission(self):
 """完整任务演示"""
 print("\n 演示6: 完整任务执行")
 print("=" * 40)
 
 try:
 print(" 任务规划...")
 
 # 复合任务：巡航 → 数据采集 → 定点保持
 mission_phases = [
 {
 'name': '巡航阶段',
 'type': 'trajectory',
 'waypoints': [
 Waypoint(0, 0, -1, 0),
 Waypoint(3, 0, -1, 0),
 Waypoint(3, 3, -2, math.pi/2),
 Waypoint(0, 3, -2, math.pi)
 ],
 'duration': 30
 },
 {
 'name': '数据采集阶段',
 'type': 'station_keeping',
 'position': [1.5, 1.5, -2],
 'heading': 0,
 'duration': 20
 },
 {
 'name': '返航阶段',
 'type': 'trajectory',
 'waypoints': [
 Waypoint(1.5, 1.5, -2, 0),
 Waypoint(0, 0, -1, 0)
 ],
 'duration': 15
 }
 ]
 
 print(f" 任务包含{len(mission_phases)}个阶段")
 
 # 执行任务
 total_mission_data = []
 mission_start_time = time.time()
 
 for phase_idx, phase in enumerate(mission_phases):
 print(f"\n 阶段 {phase_idx + 1}: {phase['name']}")
 phase_start_time = time.time()
 
 if phase['type'] == 'trajectory':
 # 轨迹跟踪阶段
 self.control_system.set_control_mode(ControlMode.TRAJECTORY)
 success = self.control_system.plan_mission(phase['waypoints'])
 
 if success:
 while time.time() - phase_start_time < phase['duration']:
 current_state = self.rov.get_rov_state()
 if not current_state:
 continue
 
 state_dict = {
 'x': current_state.position[0],
 'y': current_state.position[1],
 'z': current_state.position[2],
 'yaw': current_state.orientation[2]
 }
 
 control_output = self.control_system.execute_control(state_dict, {})
 pwm_values = self.rov.wrench_to_pwm(control_output)
 self.rov.set_thruster_pwm(pwm_values.tolist(), 0.1)
 
 # 记录任务数据
 mission_data = {
 'mission_time': time.time() - mission_start_time,
 'phase': phase['name'],
 'position': current_state.position.tolist(),
 'control': control_output.tolist()
 }
 total_mission_data.append(mission_data)
 
 time.sleep(0.1)
 
 elif phase['type'] == 'station_keeping':
 # 定点保持阶段
 self.control_system.set_control_mode(ControlMode.STATION_KEEPING)
 
 # 同时进行数据采集
 data_collection_thread = Thread(
 target=self._collect_mission_data,
 args=(phase['duration'],)
 )
 data_collection_thread.start()
 
 target_dict = {
 'x': phase['position'][0],
 'y': phase['position'][1],
 'z': phase['position'][2],
 'yaw': phase['heading']
 }
 
 while time.time() - phase_start_time < phase['duration']:
 current_state = self.rov.get_rov_state()
 if not current_state:
 continue
 
 state_dict = {
 'x': current_state.position[0],
 'y': current_state.position[1],
 'z': current_state.position[2],
 'yaw': current_state.orientation[2]
 }
 
 control_output = self.control_system.execute_control(state_dict, target_dict)
 pwm_values = self.rov.wrench_to_pwm(control_output)
 self.rov.set_thruster_pwm(pwm_values.tolist(), 0.1)
 
 time.sleep(0.1)
 
 data_collection_thread.join()
 
 elapsed = time.time() - phase_start_time
 print(f" {phase['name']}完成 ({elapsed:.1f}s)")
 
 self.rov.emergency_stop()
 
 # 任务总结
 total_mission_time = time.time() - mission_start_time
 print(f"\n 完整任务执行完成!")
 print(f" 任务统计:")
 print(f" 总时间: {total_mission_time:.1f}秒")
 print(f" 数据点: {len(total_mission_data)}")
 print(f" 阶段数: {len(mission_phases)}")
 
 # 保存任务数据
 self.save_mission_data(total_mission_data, "complete_mission")
 print(" 完整任务演示完成")
 
 except Exception as e:
 print(f" 完整任务演示失败: {e}")
 
 def _collect_mission_data(self, duration: float):
 """任务数据采集线程"""
 start_time = time.time()
 collected_data = []
 
 while time.time() - start_time < duration:
 # 采集图像
 rgb_image = self.rov.get_camera_image()
 depth_image = self.rov.get_depth_image()
 
 # 采集传感器数据
 rov_state = self.rov.get_rov_state()
 imu_data = self.rov.get_imu_data()
 
 if rov_state and imu_data:
 data_point = {
 'timestamp': time.time() - start_time,
 'has_rgb_image': rgb_image is not None,
 'has_depth_image': depth_image is not None,
 'position': rov_state.position.tolist(),
 'depth': rov_state.depth,
 'imu_acceleration': imu_data['acceleration']
 }
 collected_data.append(data_point)
 
 time.sleep(0.5) # 每0.5秒采集一次
 
 print(f" 数据采集完成: {len(collected_data)}个数据点")
 
 def run_demo(self, scenario_name: str = 'all'):
 """运行演示程序"""
 print(" HAAV_Sim Windows 最小实现 - 完整功能演示")
 print("=" * 60)
 
 # 初始化系统
 if not self.initialize_rov():
 return False
 
 try:
 self.demo_running = True
 
 if scenario_name == 'all':
 # 运行所有演示
 for name, demo_func in self.demo_scenarios.items():
 print(f"\n{'='*60}")
 demo_func()
 print("\n 暂停3秒...")
 time.sleep(3)
 
 elif scenario_name in self.demo_scenarios:
 # 运行指定演示
 self.demo_scenarios[scenario_name]()
 
 else:
 print(f" 未知演示场景: {scenario_name}")
 print(f"可用场景: {list(self.demo_scenarios.keys())}")
 return False
 
 print("\n 演示程序完成!")
 return True
 
 except KeyboardInterrupt:
 print("\n 用户中断演示")
 return False
 
 except Exception as e:
 print(f"\n 演示程序异常: {e}")
 return False
 
 finally:
 self.demo_running = False
 if self.rov:
 self.rov.shutdown()
 
 def save_trajectory_data(self, data: list, filename: str):
 """保存轨迹数据"""
 try:
 filepath = f"{filename}_{int(time.time())}.json"
 with open(filepath, 'w') as f:
 json.dump(data, f, indent=2, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x)
 print(f" 轨迹数据已保存: {filepath}")
 except Exception as e:
 print(f" 保存轨迹数据失败: {e}")
 
 def save_sensor_data(self, data: list, filename: str):
 """保存传感器数据"""
 try:
 filepath = f"{filename}_{int(time.time())}.json"
 with open(filepath, 'w') as f:
 json.dump(data, f, indent=2)
 print(f" 传感器数据已保存: {filepath}")
 except Exception as e:
 print(f" 保存传感器数据失败: {e}")
 
 def save_mission_data(self, data: list, filename: str):
 """保存任务数据"""
 try:
 filepath = f"{filename}_{int(time.time())}.json"
 with open(filepath, 'w') as f:
 json.dump(data, f, indent=2, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x)
 print(f" 任务数据已保存: {filepath}")
 except Exception as e:
 print(f" 保存任务数据失败: {e}")

def main():
 """主函数"""
 import argparse
 
 parser = argparse.ArgumentParser(description='HAAV_Sim完整ROV演示程序')
 parser.add_argument('--scenario', '-s', 
 choices=['all', 'basic_movement', 'trajectory_following', 
 'sensor_collection', 'station_keeping', 
 'obstacle_avoidance', 'complete_mission'],
 default='all',
 help='选择要运行的演示场景')
 
 args = parser.parse_args()
 
 # 创建并运行演示
 demo = ROVDemo()
 success = demo.run_demo(args.scenario)
 
 return 0 if success else 1

if __name__ == "__main__":
 exit(main())