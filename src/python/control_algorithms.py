#!/usr/bin/env python3
"""
完整的ROV控制算法实现
包含PID控制、轨迹规划、路径跟踪、障碍物避让等高级算法
"""

import numpy as np
import math
import time
from typing import List, Tuple, Optional, Dict, Callable
from dataclasses import dataclass
from enum import Enum
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

class ControlMode(Enum):
 """控制模式"""
 MANUAL = "manual"
 POSITION = "position"
 VELOCITY = "velocity"
 TRAJECTORY = "trajectory"
 STATION_KEEPING = "station_keeping"

@dataclass
class Waypoint:
 """航点定义"""
 x: float
 y: float
 z: float
 yaw: float
 velocity: float = 1.0
 tolerance: float = 0.5
 hold_time: float = 0.0

@dataclass
class PIDGains:
 """PID参数"""
 kp: float
 ki: float
 kd: float
 
class PIDController:
 """多维PID控制器"""
 
 def __init__(self, gains: PIDGains, output_limits: Tuple[float, float] = (-1, 1)):
 self.gains = gains
 self.output_limits = output_limits
 
 # 状态变量
 self.error_integral = 0.0
 self.error_prev = 0.0
 self.last_time = None
 
 # 积分限制
 self.integral_limit = abs(output_limits[1]) * 0.5
 
 def update(self, setpoint: float, measurement: float, dt: Optional[float] = None) -> float:
 """更新PID控制器"""
 current_time = time.time()
 
 if self.last_time is None:
 self.last_time = current_time
 return 0.0
 
 if dt is None:
 dt = current_time - self.last_time
 
 if dt <= 0:
 return 0.0
 
 # 计算误差
 error = setpoint - measurement
 
 # 积分项（带抗积分饱和）
 self.error_integral += error * dt
 self.error_integral = np.clip(self.error_integral, -self.integral_limit, self.integral_limit)
 
 # 微分项
 error_derivative = (error - self.error_prev) / dt
 
 # PID输出
 output = (self.gains.kp * error + 
 self.gains.ki * self.error_integral + 
 self.gains.kd * error_derivative)
 
 # 输出限制
 output = np.clip(output, self.output_limits[0], self.output_limits[1])
 
 # 更新状态
 self.error_prev = error
 self.last_time = current_time
 
 return output
 
 def reset(self):
 """重置PID控制器"""
 self.error_integral = 0.0
 self.error_prev = 0.0
 self.last_time = None

class MultiDimensionalPID:
 """多维PID控制器"""
 
 def __init__(self, dimensions: int, gains: List[PIDGains], output_limits: List[Tuple[float, float]]):
 self.dimensions = dimensions
 self.controllers = []
 
 for i in range(dimensions):
 controller = PIDController(gains[i], output_limits[i])
 self.controllers.append(controller)
 
 def update(self, setpoints: np.ndarray, measurements: np.ndarray, dt: Optional[float] = None) -> np.ndarray:
 """更新多维PID"""
 outputs = []
 for i in range(self.dimensions):
 output = self.controllers[i].update(setpoints[i], measurements[i], dt)
 outputs.append(output)
 return np.array(outputs)
 
 def reset(self):
 """重置所有控制器"""
 for controller in self.controllers:
 controller.reset()

class TrajectoryPlanner:
 """轨迹规划器"""
 
 def __init__(self, max_velocity: float = 2.0, max_acceleration: float = 1.0):
 self.max_velocity = max_velocity
 self.max_acceleration = max_acceleration
 
 def plan_linear_trajectory(self, start: np.ndarray, end: np.ndarray, 
 total_time: float, num_points: int = 100) -> List[np.ndarray]:
 """规划直线轨迹"""
 trajectory = []
 
 for i in range(num_points):
 t = i / (num_points - 1)
 point = start + t * (end - start)
 trajectory.append(point)
 
 return trajectory
 
 def plan_circular_trajectory(self, center: np.ndarray, radius: float, 
 start_angle: float, end_angle: float,
 z_level: float, num_points: int = 100) -> List[np.ndarray]:
 """规划圆形轨迹"""
 trajectory = []
 
 angle_range = end_angle - start_angle
 
 for i in range(num_points):
 t = i / (num_points - 1)
 angle = start_angle + t * angle_range
 
 x = center[0] + radius * math.cos(angle)
 y = center[1] + radius * math.sin(angle)
 z = z_level
 
 trajectory.append(np.array([x, y, z]))
 
 return trajectory
 
 def plan_spiral_trajectory(self, center: np.ndarray, start_radius: float,
 end_radius: float, start_z: float, end_z: float,
 num_turns: float, num_points: int = 200) -> List[np.ndarray]:
 """规划螺旋轨迹"""
 trajectory = []
 
 for i in range(num_points):
 t = i / (num_points - 1)
 
 # 半径线性变化
 radius = start_radius + t * (end_radius - start_radius)
 
 # 角度变化
 angle = t * num_turns * 2 * math.pi
 
 # 深度线性变化
 z = start_z + t * (end_z - start_z)
 
 x = center[0] + radius * math.cos(angle)
 y = center[1] + radius * math.sin(angle)
 
 trajectory.append(np.array([x, y, z]))
 
 return trajectory
 
 def plan_smooth_trajectory(self, waypoints: List[np.ndarray], 
 smoothing_factor: float = 0.1) -> List[np.ndarray]:
 """规划平滑轨迹（样条插值）"""
 if len(waypoints) < 2:
 return waypoints
 
 # 参数化路径
 waypoints_array = np.array(waypoints)
 distances = np.cumsum([0] + [np.linalg.norm(waypoints_array[i+1] - waypoints_array[i]) 
 for i in range(len(waypoints)-1)])
 
 # 三次样条插值
 cs_x = CubicSpline(distances, waypoints_array[:, 0])
 cs_y = CubicSpline(distances, waypoints_array[:, 1])
 cs_z = CubicSpline(distances, waypoints_array[:, 2])
 
 # 生成平滑轨迹点
 total_distance = distances[-1]
 num_points = max(100, int(total_distance * 10)) # 每米10个点
 
 smooth_distances = np.linspace(0, total_distance, num_points)
 smooth_trajectory = []
 
 for d in smooth_distances:
 point = np.array([cs_x(d), cs_y(d), cs_z(d)])
 smooth_trajectory.append(point)
 
 return smooth_trajectory
 
 def plan_minimum_time_trajectory(self, waypoints: List[np.ndarray]) -> Tuple[List[np.ndarray], List[float]]:
 """最短时间轨迹规划"""
 if len(waypoints) < 2:
 return waypoints, [0.0]
 
 trajectory = []
 timestamps = [0.0]
 current_time = 0.0
 
 for i in range(len(waypoints) - 1):
 start = waypoints[i]
 end = waypoints[i + 1]
 distance = np.linalg.norm(end - start)
 
 # 计算最优时间（考虑速度和加速度限制）
 if distance == 0:
 segment_time = 0.0
 else:
 # 简化的最优时间计算
 segment_time = max(distance / self.max_velocity,
 math.sqrt(2 * distance / self.max_acceleration))
 
 # 生成段轨迹
 segment_points = 20
 for j in range(segment_points):
 if i == 0 or j > 0: # 避免重复添加航点
 t = j / (segment_points - 1)
 point = start + t * (end - start)
 trajectory.append(point)
 timestamps.append(current_time + t * segment_time)
 
 current_time += segment_time
 
 return trajectory, timestamps

class PathFollowingController:
 """路径跟踪控制器"""
 
 def __init__(self, look_ahead_distance: float = 1.0):
 self.look_ahead_distance = look_ahead_distance
 self.path = []
 self.current_target_index = 0
 
 def set_path(self, path: List[np.ndarray]):
 """设置路径"""
 self.path = path
 self.current_target_index = 0
 
 def get_target_point(self, current_position: np.ndarray) -> Optional[np.ndarray]:
 """获取目标跟踪点（纯追踪算法）"""
 if not self.path or self.current_target_index >= len(self.path):
 return None
 
 # 寻找前瞻距离内的目标点
 for i in range(self.current_target_index, len(self.path)):
 distance = np.linalg.norm(self.path[i] - current_position)
 
 if distance >= self.look_ahead_distance:
 self.current_target_index = i
 return self.path[i]
 
 # 如果没有找到，返回路径终点
 return self.path[-1]
 
 def calculate_cross_track_error(self, current_position: np.ndarray) -> float:
 """计算横向跟踪误差"""
 if len(self.path) < 2 or self.current_target_index >= len(self.path) - 1:
 return 0.0
 
 # 当前路径段
 p1 = self.path[self.current_target_index]
 p2 = self.path[min(self.current_target_index + 1, len(self.path) - 1)]
 
 # 计算点到直线的距离
 line_vec = p2 - p1
 point_vec = current_position - p1
 
 if np.linalg.norm(line_vec) == 0:
 return np.linalg.norm(point_vec)
 
 # 投影长度
 proj_length = np.dot(point_vec, line_vec) / np.linalg.norm(line_vec)
 
 # 垂直距离（横向误差）
 cross_track_error = np.linalg.norm(point_vec - proj_length * line_vec / np.linalg.norm(line_vec))
 
 return cross_track_error

class ObstacleAvoidance:
 """障碍物避让"""
 
 def __init__(self, safety_distance: float = 2.0):
 self.safety_distance = safety_distance
 self.obstacles = [] # 障碍物列表 [(x, y, z, radius)]
 
 def add_obstacle(self, position: np.ndarray, radius: float):
 """添加障碍物"""
 obstacle = (*position, radius)
 self.obstacles.append(obstacle)
 
 def clear_obstacles(self):
 """清除所有障碍物"""
 self.obstacles = []
 
 def check_collision(self, position: np.ndarray) -> bool:
 """检查是否与障碍物碰撞"""
 for obs_x, obs_y, obs_z, radius in self.obstacles:
 obs_pos = np.array([obs_x, obs_y, obs_z])
 distance = np.linalg.norm(position - obs_pos)
 
 if distance < radius + self.safety_distance:
 return True
 
 return False
 
 def calculate_repulsive_force(self, position: np.ndarray, goal: np.ndarray) -> np.ndarray:
 """计算人工势场的排斥力"""
 total_force = np.zeros(3)
 
 for obs_x, obs_y, obs_z, radius in self.obstacles:
 obs_pos = np.array([obs_x, obs_y, obs_z])
 distance = np.linalg.norm(position - obs_pos)
 
 if distance < radius + self.safety_distance * 2:
 # 排斥力方向（远离障碍物）
 if distance > 0:
 direction = (position - obs_pos) / distance
 
 # 排斥力大小（距离越近力越大）
 force_magnitude = 1.0 / (distance - radius + 0.1)**2
 force_magnitude = min(force_magnitude, 10.0) # 限制最大力
 
 total_force += force_magnitude * direction
 
 # 吸引力（朝向目标）
 goal_distance = np.linalg.norm(goal - position)
 if goal_distance > 0:
 goal_attraction = 0.5 * (goal - position) / goal_distance
 total_force += goal_attraction
 
 return total_force
 
 def plan_avoiding_path(self, start: np.ndarray, goal: np.ndarray, 
 num_iterations: int = 100) -> List[np.ndarray]:
 """规划避障路径（人工势场法）"""
 path = [start.copy()]
 current_pos = start.copy()
 step_size = 0.1
 
 for _ in range(num_iterations):
 # 计算合力
 force = self.calculate_repulsive_force(current_pos, goal)
 
 # 归一化并应用步长
 if np.linalg.norm(force) > 0:
 force_normalized = force / np.linalg.norm(force)
 next_pos = current_pos + step_size * force_normalized
 else:
 next_pos = current_pos
 
 # 检查是否到达目标
 if np.linalg.norm(next_pos - goal) < 0.2:
 path.append(goal)
 break
 
 # 检查是否陷入局部最小值
 if len(path) > 10 and np.linalg.norm(next_pos - path[-10]) < 0.1:
 # 添加随机扰动
 random_direction = np.random.randn(3)
 random_direction = random_direction / np.linalg.norm(random_direction)
 next_pos += 0.5 * random_direction
 
 current_pos = next_pos
 path.append(current_pos.copy())
 
 return path

class StationKeepingController:
 """定点保持控制器"""
 
 def __init__(self, position_tolerance: float = 0.5, heading_tolerance: float = 0.1):
 self.position_tolerance = position_tolerance
 self.heading_tolerance = heading_tolerance
 self.target_position = np.zeros(3)
 self.target_heading = 0.0
 
 # PID控制器
 position_gains = [
 PIDGains(kp=2.0, ki=0.1, kd=1.0), # X
 PIDGains(kp=2.0, ki=0.1, kd=1.0), # Y 
 PIDGains(kp=3.0, ki=0.2, kd=1.5), # Z
 ]
 
 heading_gains = PIDGains(kp=1.5, ki=0.05, kd=0.8)
 
 self.position_controller = MultiDimensionalPID(
 3, position_gains, [(-1, 1)] * 3
 )
 self.heading_controller = PIDController(heading_gains, (-1, 1))
 
 def set_target(self, position: np.ndarray, heading: float):
 """设置目标位置和航向"""
 self.target_position = position.copy()
 self.target_heading = heading
 
 def update(self, current_position: np.ndarray, current_heading: float) -> Tuple[np.ndarray, float]:
 """更新定点保持控制"""
 # 位置控制
 position_control = self.position_controller.update(
 self.target_position, current_position
 )
 
 # 航向控制（处理角度包络）
 heading_error = self.normalize_angle(self.target_heading - current_heading)
 heading_control = self.heading_controller.update(0.0, -heading_error)
 
 return position_control, heading_control
 
 def is_on_station(self, current_position: np.ndarray, current_heading: float) -> bool:
 """检查是否到达定点"""
 position_error = np.linalg.norm(current_position - self.target_position)
 heading_error = abs(self.normalize_angle(self.target_heading - current_heading))
 
 return (position_error < self.position_tolerance and 
 heading_error < self.heading_tolerance)
 
 @staticmethod
 def normalize_angle(angle: float) -> float:
 """角度归一化"""
 while angle > math.pi:
 angle -= 2 * math.pi
 while angle < -math.pi:
 angle += 2 * math.pi
 return angle

class AdaptiveController:
 """自适应控制器"""
 
 def __init__(self, adaptation_rate: float = 0.01):
 self.adaptation_rate = adaptation_rate
 self.parameter_estimates = {}
 self.reference_model = None
 
 def update_parameters(self, error: np.ndarray, regressor: np.ndarray):
 """更新参数估计（最简单的梯度自适应）"""
 parameter_update = -self.adaptation_rate * np.outer(error, regressor)
 
 if 'theta' not in self.parameter_estimates:
 self.parameter_estimates['theta'] = np.zeros((len(error), len(regressor)))
 
 self.parameter_estimates['theta'] += parameter_update

class ROVControlSystem:
 """完整ROV控制系统"""
 
 def __init__(self):
 self.control_mode = ControlMode.MANUAL
 
 # 控制器组件
 self.trajectory_planner = TrajectoryPlanner()
 self.path_follower = PathFollowingController()
 self.obstacle_avoidance = ObstacleAvoidance()
 self.station_keeper = StationKeepingController()
 
 # 主控制器
 self.main_controller = self._create_main_controller()
 
 # 状态变量
 self.current_trajectory = []
 self.current_waypoint_index = 0
 self.control_active = False
 
 def _create_main_controller(self) -> MultiDimensionalPID:
 """创建主PID控制器"""
 gains = [
 PIDGains(kp=3.0, ki=0.2, kd=1.5), # X
 PIDGains(kp=3.0, ki=0.2, kd=1.5), # Y
 PIDGains(kp=4.0, ki=0.3, kd=2.0), # Z
 PIDGains(kp=2.0, ki=0.1, kd=1.0), # Yaw
 ]
 
 return MultiDimensionalPID(4, gains, [(-1, 1)] * 4)
 
 def set_control_mode(self, mode: ControlMode):
 """设置控制模式"""
 self.control_mode = mode
 self.main_controller.reset()
 print(f" 控制模式切换到: {mode.value}")
 
 def plan_mission(self, waypoints: List[Waypoint]) -> bool:
 """规划任务轨迹"""
 try:
 # 转换航点为数组
 waypoint_positions = [np.array([wp.x, wp.y, wp.z]) for wp in waypoints]
 
 # 规划平滑轨迹
 self.current_trajectory = self.trajectory_planner.plan_smooth_trajectory(waypoint_positions)
 self.current_waypoint_index = 0
 
 print(f" 任务规划完成，共{len(self.current_trajectory)}个轨迹点")
 return True
 
 except Exception as e:
 print(f" 任务规划失败: {e}")
 return False
 
 def execute_control(self, current_state: Dict, target_state: Dict) -> np.ndarray:
 """执行控制算法"""
 try:
 current_pos = np.array([current_state['x'], current_state['y'], current_state['z']])
 current_yaw = current_state['yaw']
 
 if self.control_mode == ControlMode.POSITION:
 return self._position_control(current_pos, current_yaw, target_state)
 
 elif self.control_mode == ControlMode.TRAJECTORY:
 return self._trajectory_control(current_pos, current_yaw)
 
 elif self.control_mode == ControlMode.STATION_KEEPING:
 return self._station_keeping_control(current_pos, current_yaw, target_state)
 
 else:
 return np.zeros(4) # Manual mode
 
 except Exception as e:
 print(f" 控制执行错误: {e}")
 return np.zeros(4)
 
 def _position_control(self, current_pos: np.ndarray, current_yaw: float, target: Dict) -> np.ndarray:
 """位置控制"""
 target_pos = np.array([target['x'], target['y'], target['z']])
 target_yaw = target.get('yaw', 0.0)
 
 # 检查障碍物
 if self.obstacle_avoidance.check_collision(target_pos):
 print(" 目标位置有障碍物，重新规划路径")
 avoiding_path = self.obstacle_avoidance.plan_avoiding_path(current_pos, target_pos)
 if avoiding_path:
 target_pos = avoiding_path[min(1, len(avoiding_path)-1)]
 
 # PID控制
 current_state = np.array([current_pos[0], current_pos[1], current_pos[2], current_yaw])
 target_state = np.array([target_pos[0], target_pos[1], target_pos[2], target_yaw])
 
 control_output = self.main_controller.update(target_state, current_state)
 
 return control_output
 
 def _trajectory_control(self, current_pos: np.ndarray, current_yaw: float) -> np.ndarray:
 """轨迹跟踪控制"""
 if not self.current_trajectory:
 return np.zeros(4)
 
 # 获取当前目标点
 target_point = self.path_follower.get_target_point(current_pos)
 if target_point is None:
 return np.zeros(4)
 
 # 计算目标航向
 if self.current_waypoint_index < len(self.current_trajectory) - 1:
 next_point = self.current_trajectory[self.current_waypoint_index + 1]
 direction = next_point - target_point
 target_yaw = math.atan2(direction[1], direction[0])
 else:
 target_yaw = current_yaw
 
 # 执行位置控制
 target_state = {'x': target_point[0], 'y': target_point[1], 'z': target_point[2], 'yaw': target_yaw}
 return self._position_control(current_pos, current_yaw, target_state)
 
 def _station_keeping_control(self, current_pos: np.ndarray, current_yaw: float, target: Dict) -> np.ndarray:
 """定点保持控制"""
 target_pos = np.array([target['x'], target['y'], target['z']])
 target_heading = target.get('yaw', 0.0)
 
 self.station_keeper.set_target(target_pos, target_heading)
 position_control, heading_control = self.station_keeper.update(current_pos, current_yaw)
 
 return np.array([position_control[0], position_control[1], position_control[2], heading_control])
 
 def get_control_status(self) -> Dict:
 """获取控制状态信息"""
 return {
 'control_mode': self.control_mode.value,
 'trajectory_points': len(self.current_trajectory),
 'current_waypoint': self.current_waypoint_index,
 'control_active': self.control_active,
 'obstacles': len(self.obstacle_avoidance.obstacles)
 }

# 使用示例和测试函数
def test_control_algorithms():
 """测试控制算法"""
 print(" 控制算法测试")
 print("=" * 30)
 
 # 创建控制系统
 control_system = ROVControlSystem()
 
 # 测试轨迹规划
 waypoints = [
 Waypoint(0, 0, -1, 0),
 Waypoint(5, 0, -1, 0),
 Waypoint(5, 5, -2, math.pi/2),
 Waypoint(0, 5, -2, math.pi),
 Waypoint(0, 0, -1, 0)
 ]
 
 success = control_system.plan_mission(waypoints)
 print(f"轨迹规划: {'' if success else ''}")
 
 # 测试PID控制
 control_system.set_control_mode(ControlMode.POSITION)
 
 current_state = {'x': 0, 'y': 0, 'z': -0.5, 'yaw': 0}
 target_state = {'x': 1, 'y': 1, 'z': -1, 'yaw': math.pi/4}
 
 control_output = control_system.execute_control(current_state, target_state)
 print(f"位置控制输出: {control_output}")
 
 # 测试定点保持
 control_system.set_control_mode(ControlMode.STATION_KEEPING)
 control_output = control_system.execute_control(current_state, target_state)
 print(f"定点保持输出: {control_output}")
 
 print(" 控制算法测试完成")

if __name__ == "__main__":
 test_control_algorithms()