#!/usr/bin/env python3
"""
ROV轨迹演示程序集
包含多种复杂轨迹的演示：螺旋上升、八字轨迹、正弦波、3D立体轨迹等
"""

import sys
import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
from typing import List, Tuple, Optional
from dataclasses import dataclass
from threading import Thread, Event

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'python'))

from rov_client import ROVController
from control_algorithms import ROVControlSystem, Waypoint, ControlMode, TrajectoryPlanner

@dataclass
class TrajectoryPoint:
    """轨迹点定义"""
    x: float
    y: float
    z: float
    yaw: float
    velocity: float = 1.0
    timestamp: float = 0.0

class TrajectoryGenerator:
    """高级轨迹生成器"""
    
    def __init__(self):
        self.dt = 0.1  # 时间步长
        
    def generate_spiral_ascent(self, center: np.ndarray, start_radius: float, 
                             end_radius: float, start_depth: float, end_depth: float,
                             turns: float, duration: float) -> List[TrajectoryPoint]:
        """生成螺旋上升轨迹"""
        points = []
        num_points = int(duration / self.dt)
        
        for i in range(num_points):
            t = i / (num_points - 1)  # 0 到 1
            
            # 半径从start_radius到end_radius
            radius = start_radius + t * (end_radius - start_radius)
            
            # 角度变化
            angle = t * turns * 2 * math.pi
            
            # 深度从start_depth到end_depth (上升为负值减小)
            depth = start_depth + t * (end_depth - start_depth)
            
            # 计算位置
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            z = depth
            
            # 计算朝向 (沿运动方向)
            if i < num_points - 1:
                next_angle = (i + 1) / (num_points - 1) * turns * 2 * math.pi
                yaw = next_angle + math.pi / 2  # 垂直于半径方向
            else:
                yaw = angle + math.pi / 2
            
            point = TrajectoryPoint(x, y, z, yaw, 1.0, i * self.dt)
            points.append(point)
            
        return points
    
    def generate_figure_eight(self, center: np.ndarray, width: float, height: float,
                            depth: float, duration: float) -> List[TrajectoryPoint]:
        """生成八字轨迹"""
        points = []
        num_points = int(duration / self.dt)
        
        for i in range(num_points):
            t = i / (num_points - 1) * 2 * math.pi  # 0 到 2π
            
            # 八字轨迹参数方程
            x = center[0] + width * math.sin(t)
            y = center[1] + height * math.sin(2 * t)  # 频率加倍产生8字
            z = depth
            
            # 计算切线方向作为朝向
            dx_dt = width * math.cos(t)
            dy_dt = 2 * height * math.cos(2 * t)
            yaw = math.atan2(dy_dt, dx_dt)
            
            point = TrajectoryPoint(x, y, z, yaw, 1.5, i * self.dt)
            points.append(point)
            
        return points
    
    def generate_sine_wave(self, start: np.ndarray, end: np.ndarray, 
                          amplitude: float, frequency: float, 
                          duration: float) -> List[TrajectoryPoint]:
        """生成正弦波轨迹"""
        points = []
        num_points = int(duration / self.dt)
        
        # 计算主方向
        direction = end - start
        distance = np.linalg.norm(direction)
        if distance == 0:
            return points
            
        main_dir = direction / distance
        
        # 计算垂直方向 (在XY平面内)
        perp_dir = np.array([-main_dir[1], main_dir[0], 0])
        if np.linalg.norm(perp_dir) == 0:
            perp_dir = np.array([1, 0, 0])  # 如果主方向是Z轴，使用X轴作为垂直方向
        else:
            perp_dir = perp_dir / np.linalg.norm(perp_dir)
        
        for i in range(num_points):
            t = i / (num_points - 1)  # 0 到 1
            
            # 沿主方向的位置
            base_pos = start + t * direction
            
            # 正弦波偏移
            sine_offset = amplitude * math.sin(frequency * t * 2 * math.pi)
            final_pos = base_pos + sine_offset * perp_dir
            
            # 计算朝向
            if i < num_points - 1:
                next_t = (i + 1) / (num_points - 1)
                next_sine = amplitude * math.sin(frequency * next_t * 2 * math.pi)
                next_pos = start + next_t * direction + next_sine * perp_dir
                
                move_dir = next_pos - final_pos
                if np.linalg.norm(move_dir) > 0:
                    yaw = math.atan2(move_dir[1], move_dir[0])
                else:
                    yaw = 0
            else:
                yaw = math.atan2(main_dir[1], main_dir[0])
            
            point = TrajectoryPoint(final_pos[0], final_pos[1], final_pos[2], 
                                  yaw, 1.2, i * self.dt)
            points.append(point)
            
        return points
    
    def generate_helix_3d(self, center: np.ndarray, radius: float, pitch: float,
                         turns: float, duration: float) -> List[TrajectoryPoint]:
        """生成3D螺旋线轨迹"""
        points = []
        num_points = int(duration / self.dt)
        
        total_height = turns * pitch
        
        for i in range(num_points):
            t = i / (num_points - 1)  # 0 到 1
            
            # 角度
            angle = t * turns * 2 * math.pi
            
            # 3D螺旋位置
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)  
            z = center[2] - t * total_height  # 向下螺旋
            
            # 计算切线方向
            yaw = angle + math.pi / 2  # 切线方向
            
            point = TrajectoryPoint(x, y, z, yaw, 1.0, i * self.dt)
            points.append(point)
            
        return points
    
    def generate_lemniscate_3d(self, center: np.ndarray, scale: float, 
                             depth: float, duration: float) -> List[TrajectoryPoint]:
        """生成3D双纽线轨迹 (∞形状)"""
        points = []
        num_points = int(duration / self.dt)
        
        for i in range(num_points):
            t = i / (num_points - 1) * 4 * math.pi  # 0 到 4π，完整的双纽线
            
            # 双纽线参数方程
            cos_t = math.cos(t)
            sin_t = math.sin(t)
            denominator = 1 + sin_t ** 2
            
            if denominator != 0:
                x = center[0] + scale * cos_t / denominator
                y = center[1] + scale * sin_t * cos_t / denominator
                z = depth + 0.5 * scale * math.sin(2 * t) / denominator  # 添加Z轴变化
            else:
                x, y, z = center[0], center[1], depth
            
            # 计算朝向
            if i < num_points - 1:
                next_t = (i + 1) / (num_points - 1) * 4 * math.pi
                next_cos = math.cos(next_t)
                next_sin = math.sin(next_t)
                next_denom = 1 + next_sin ** 2
                
                if next_denom != 0:
                    next_x = center[0] + scale * next_cos / next_denom
                    next_y = center[1] + scale * next_sin * next_cos / next_denom
                    
                    dx = next_x - x
                    dy = next_y - y
                    yaw = math.atan2(dy, dx) if dx != 0 or dy != 0 else 0
                else:
                    yaw = 0
            else:
                yaw = 0
            
            point = TrajectoryPoint(x, y, z, yaw, 0.8, i * self.dt)
            points.append(point)
            
        return points
    
    def generate_rose_curve(self, center: np.ndarray, radius: float, petals: int,
                          depth: float, duration: float) -> List[TrajectoryPoint]:
        """生成玫瑰线轨迹 (花瓣形状)"""
        points = []
        num_points = int(duration / self.dt)
        
        for i in range(num_points):
            t = i / (num_points - 1) * 2 * math.pi  # 0 到 2π
            
            # 玫瑰线方程: r = radius * cos(petals * θ)
            r = radius * abs(math.cos(petals * t))
            
            x = center[0] + r * math.cos(t)
            y = center[1] + r * math.sin(t)
            z = depth
            
            # 计算切线方向
            dr_dt = -radius * petals * math.sin(petals * t) * math.copysign(1, math.cos(petals * t))
            dx_dt = dr_dt * math.cos(t) - r * math.sin(t)
            dy_dt = dr_dt * math.sin(t) + r * math.cos(t)
            
            yaw = math.atan2(dy_dt, dx_dt) if dx_dt != 0 or dy_dt != 0 else t
            
            point = TrajectoryPoint(x, y, z, yaw, 1.0, i * self.dt)
            points.append(point)
            
        return points
    
    def generate_cloverleaf(self, center: np.ndarray, size: float,
                          depth: float, duration: float) -> List[TrajectoryPoint]:
        """生成四叶草轨迹"""
        points = []
        num_points = int(duration / self.dt)
        
        for i in range(num_points):
            t = i / (num_points - 1) * 2 * math.pi  # 0 到 2π
            
            # 四叶草参数方程
            r = size * abs(math.cos(2 * t))
            
            x = center[0] + r * math.cos(t)
            y = center[1] + r * math.sin(t)
            z = depth + 0.2 * size * math.sin(4 * t)  # 添加轻微的Z轴变化
            
            # 计算朝向
            yaw = t + math.pi / 2
            
            point = TrajectoryPoint(x, y, z, yaw, 1.0, i * self.dt)
            points.append(point)
            
        return points

class TrajectoryDemoRunner:
    """轨迹演示运行器"""
    
    def __init__(self):
        self.rov = None
        self.control_system = ROVControlSystem()
        self.trajectory_generator = TrajectoryGenerator()
        self.demo_running = False
        self.trajectory_data = []
        
        # 演示配置
        self.demos = {
            'spiral_ascent': {
                'name': '螺旋上升轨迹',
                'description': 'ROV从深水螺旋上升至浅水区域',
                'duration': 60,
                'func': self.demo_spiral_ascent
            },
            'figure_eight': {
                'name': '八字轨迹',
                'description': 'ROV执行平面八字飞行',
                'duration': 45,
                'func': self.demo_figure_eight
            },
            'sine_wave': {
                'name': '正弦波轨迹',
                'description': 'ROV沿正弦波路径移动',
                'duration': 40,
                'func': self.demo_sine_wave
            },
            'helix_3d': {
                'name': '3D螺旋轨迹',
                'description': 'ROV执行三维螺旋运动',
                'duration': 50,
                'func': self.demo_helix_3d
            },
            'lemniscate_3d': {
                'name': '3D双纽线',
                'description': 'ROV执行立体无穷符号轨迹',
                'duration': 55,
                'func': self.demo_lemniscate_3d
            },
            'rose_curve': {
                'name': '玫瑰线轨迹',
                'description': 'ROV执行花瓣状轨迹',
                'duration': 45,
                'func': self.demo_rose_curve
            },
            'cloverleaf': {
                'name': '四叶草轨迹',
                'description': 'ROV执行四叶草形状轨迹',
                'duration': 40,
                'func': self.demo_cloverleaf
            },
            'complex_mission': {
                'name': '复合轨迹任务',
                'description': '多种轨迹组合的复杂任务',
                'duration': 120,
                'func': self.demo_complex_mission
            }
        }
    
    def initialize(self) -> bool:
        """初始化ROV系统"""
        try:
            print("🚁 初始化ROV轨迹演示系统...")
            self.rov = ROVController()
            
            if self.rov.initialize():
                print("✅ ROV系统初始化成功")
                
                # 显示初始状态
                state = self.rov.get_rov_state()
                if state:
                    print(f"📍 初始位置: ({state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f})")
                    print(f"🌊 当前深度: {state.depth:.2f}m")
                
                return True
            else:
                print("❌ ROV初始化失败")
                return False
                
        except Exception as e:
            print(f"❌ 初始化异常: {e}")
            return False
    
    def demo_spiral_ascent(self):
        """螺旋上升轨迹演示"""
        print("\n🌪️ 螺旋上升轨迹演示")
        print("=" * 40)
        
        try:
            # 获取当前位置作为中心
            current_state = self.rov.get_rov_state()
            center = current_state.position
            
            # 生成螺旋上升轨迹
            trajectory = self.trajectory_generator.generate_spiral_ascent(
                center=center,
                start_radius=0.5,    # 起始半径0.5米
                end_radius=2.0,      # 结束半径2米
                start_depth=center[2], # 当前深度
                end_depth=center[2] + 3,  # 上升3米
                turns=3.0,           # 3圈螺旋
                duration=60.0        # 60秒
            )
            
            print(f"✅ 生成螺旋轨迹: {len(trajectory)}个点")
            print(f"📐 起始半径: 0.5m → 结束半径: 2.0m")
            print(f"📊 螺旋圈数: 3圈")
            print(f"⬆️ 上升高度: 3.0m")
            
            # 执行轨迹
            self.execute_trajectory(trajectory, "螺旋上升")
            
        except Exception as e:
            print(f"❌ 螺旋上升演示失败: {e}")
    
    def demo_figure_eight(self):
        """八字轨迹演示"""
        print("\n♾️ 八字轨迹演示")
        print("=" * 40)
        
        try:
            current_state = self.rov.get_rov_state()
            center = current_state.position
            
            # 生成八字轨迹
            trajectory = self.trajectory_generator.generate_figure_eight(
                center=center,
                width=3.0,      # 宽度3米
                height=2.0,     # 高度2米  
                depth=center[2], # 保持当前深度
                duration=45.0   # 45秒
            )
            
            print(f"✅ 生成八字轨迹: {len(trajectory)}个点")
            print(f"📏 轨迹尺寸: 3.0m × 2.0m")
            print(f"🎯 轨迹深度: {center[2]:.1f}m")
            
            # 执行轨迹
            self.execute_trajectory(trajectory, "八字轨迹")
            
        except Exception as e:
            print(f"❌ 八字轨迹演示失败: {e}")
    
    def demo_sine_wave(self):
        """正弦波轨迹演示"""
        print("\n〰️ 正弦波轨迹演示")
        print("=" * 40)
        
        try:
            current_state = self.rov.get_rov_state()
            start = current_state.position
            end = start + np.array([8.0, 2.0, -1.0])  # 终点偏移
            
            # 生成正弦波轨迹
            trajectory = self.trajectory_generator.generate_sine_wave(
                start=start,
                end=end,
                amplitude=1.5,    # 振幅1.5米
                frequency=2.0,    # 频率2Hz
                duration=40.0     # 40秒
            )
            
            print(f"✅ 生成正弦波轨迹: {len(trajectory)}个点")
            print(f"📊 波形参数: 振幅={1.5}m, 频率={2.0}Hz")
            print(f"📍 起点: ({start[0]:.1f}, {start[1]:.1f}, {start[2]:.1f})")
            print(f"📍 终点: ({end[0]:.1f}, {end[1]:.1f}, {end[2]:.1f})")
            
            # 执行轨迹
            self.execute_trajectory(trajectory, "正弦波")
            
        except Exception as e:
            print(f"❌ 正弦波轨迹演示失败: {e}")
    
    def demo_helix_3d(self):
        """3D螺旋轨迹演示"""
        print("\n🧬 3D螺旋轨迹演示")
        print("=" * 40)
        
        try:
            current_state = self.rov.get_rov_state()
            center = current_state.position
            
            # 生成3D螺旋轨迹
            trajectory = self.trajectory_generator.generate_helix_3d(
                center=center,
                radius=2.0,     # 半径2米
                pitch=1.0,      # 螺距1米  
                turns=4.0,      # 4圈螺旋
                duration=50.0   # 50秒
            )
            
            print(f"✅ 生成3D螺旋轨迹: {len(trajectory)}个点")
            print(f"📐 螺旋参数: 半径={2.0}m, 螺距={1.0}m")
            print(f"🌪️ 螺旋圈数: 4圈")
            print(f"📏 总高度: {4.0 * 1.0}m")
            
            # 执行轨迹
            self.execute_trajectory(trajectory, "3D螺旋")
            
        except Exception as e:
            print(f"❌ 3D螺旋轨迹演示失败: {e}")
    
    def demo_lemniscate_3d(self):
        """3D双纽线轨迹演示"""
        print("\n∞ 3D双纽线轨迹演示")
        print("=" * 40)
        
        try:
            current_state = self.rov.get_rov_state()
            center = current_state.position
            
            # 生成3D双纽线轨迹
            trajectory = self.trajectory_generator.generate_lemniscate_3d(
                center=center,
                scale=2.5,        # 缩放因子2.5米
                depth=center[2],  # 基准深度
                duration=55.0     # 55秒
            )
            
            print(f"✅ 生成3D双纽线轨迹: {len(trajectory)}个点")
            print(f"📏 轨迹缩放: {2.5}m")
            print(f"🎯 基准深度: {center[2]:.1f}m")
            print(f"📊 轨迹特点: 立体无穷符号，包含Z轴变化")
            
            # 执行轨迹
            self.execute_trajectory(trajectory, "3D双纽线")
            
        except Exception as e:
            print(f"❌ 3D双纽线轨迹演示失败: {e}")
    
    def demo_rose_curve(self):
        """玫瑰线轨迹演示"""
        print("\n🌹 玫瑰线轨迹演示")
        print("=" * 40)
        
        try:
            current_state = self.rov.get_rov_state()
            center = current_state.position
            
            # 生成玫瑰线轨迹 (5瓣玫瑰)
            trajectory = self.trajectory_generator.generate_rose_curve(
                center=center,
                radius=2.0,       # 最大半径2米
                petals=5,         # 5个花瓣
                depth=center[2],  # 保持深度
                duration=45.0     # 45秒
            )
            
            print(f"✅ 生成玫瑰线轨迹: {len(trajectory)}个点")
            print(f"🌹 花瓣数量: 5瓣")
            print(f"📏 最大半径: {2.0}m")
            print(f"🎯 轨迹深度: {center[2]:.1f}m")
            
            # 执行轨迹
            self.execute_trajectory(trajectory, "玫瑰线")
            
        except Exception as e:
            print(f"❌ 玫瑰线轨迹演示失败: {e}")
    
    def demo_cloverleaf(self):
        """四叶草轨迹演示"""
        print("\n🍀 四叶草轨迹演示")
        print("=" * 40)
        
        try:
            current_state = self.rov.get_rov_state()
            center = current_state.position
            
            # 生成四叶草轨迹
            trajectory = self.trajectory_generator.generate_cloverleaf(
                center=center,
                size=2.0,         # 尺寸2米
                depth=center[2],  # 基准深度
                duration=40.0     # 40秒
            )
            
            print(f"✅ 生成四叶草轨迹: {len(trajectory)}个点")
            print(f"🍀 叶片尺寸: {2.0}m")
            print(f"🎯 基准深度: {center[2]:.1f}m")
            print(f"📊 轨迹特点: 四叶草形状，包含轻微Z轴变化")
            
            # 执行轨迹
            self.execute_trajectory(trajectory, "四叶草")
            
        except Exception as e:
            print(f"❌ 四叶草轨迹演示失败: {e}")
    
    def demo_complex_mission(self):
        """复合轨迹任务演示"""
        print("\n🎯 复合轨迹任务演示")
        print("=" * 40)
        
        try:
            current_state = self.rov.get_rov_state()
            start_pos = current_state.position
            
            print("📋 任务规划: 螺旋下降 → 八字巡航 → 玫瑰采样 → 直线返回")
            
            # 阶段1: 螺旋下降到工作深度
            print("\n🌪️ 阶段1: 螺旋下降")
            spiral_traj = self.trajectory_generator.generate_spiral_ascent(
                center=start_pos,
                start_radius=0.5,
                end_radius=1.5,
                start_depth=start_pos[2],
                end_depth=start_pos[2] - 2.0,  # 下降2米
                turns=2.0,
                duration=30.0
            )
            
            if spiral_traj:
                work_depth = spiral_traj[-1].z
                work_center = np.array([spiral_traj[-1].x, spiral_traj[-1].y, work_depth])
                
                self.execute_trajectory(spiral_traj, "螺旋下降")
                print("✅ 螺旋下降完成")
                
                # 短暂悬停
                time.sleep(2)
                
                # 阶段2: 八字巡航
                print("\n♾️ 阶段2: 八字巡航")
                eight_traj = self.trajectory_generator.generate_figure_eight(
                    center=work_center,
                    width=2.0,
                    height=1.5,
                    depth=work_depth,
                    duration=25.0
                )
                
                self.execute_trajectory(eight_traj, "八字巡航")
                print("✅ 八字巡航完成")
                
                time.sleep(2)
                
                # 阶段3: 玫瑰线精密采样
                print("\n🌹 阶段3: 玫瑰线精密采样")
                rose_traj = self.trajectory_generator.generate_rose_curve(
                    center=work_center,
                    radius=1.0,
                    petals=3,
                    depth=work_depth,
                    duration=25.0
                )
                
                self.execute_trajectory(rose_traj, "玫瑰采样")
                print("✅ 玫瑰采样完成")
                
                time.sleep(2)
                
                # 阶段4: 直线返回起始点
                print("\n⬅️ 阶段4: 直线返回")
                current_pos = np.array([work_center[0], work_center[1], work_depth])
                return_traj = self.trajectory_generator.generate_sine_wave(
                    start=current_pos,
                    end=start_pos,
                    amplitude=0.0,  # 直线，无波动
                    frequency=0.0,
                    duration=20.0
                )
                
                self.execute_trajectory(return_traj, "直线返回")
                print("✅ 直线返回完成")
                
                print("\n🎉 复合轨迹任务完成!")
                
                # 任务统计
                total_points = len(spiral_traj) + len(eight_traj) + len(rose_traj) + len(return_traj)
                total_duration = 30 + 25 + 25 + 20
                
                print(f"📊 任务统计:")
                print(f"   总轨迹点数: {total_points}")
                print(f"   总执行时间: {total_duration}秒")
                print(f"   任务阶段数: 4个")
                print(f"   垂直位移: {abs(start_pos[2] - work_depth):.1f}m")
            
        except Exception as e:
            print(f"❌ 复合任务演示失败: {e}")
    
    def execute_trajectory(self, trajectory: List[TrajectoryPoint], 
                          trajectory_name: str):
        """执行轨迹跟踪"""
        try:
            if not trajectory:
                print("⚠️ 轨迹为空，跳过执行")
                return
            
            print(f"🚀 开始执行{trajectory_name}轨迹...")
            print(f"📊 轨迹点数: {len(trajectory)}")
            
            # 设置轨迹跟踪模式
            self.control_system.set_control_mode(ControlMode.POSITION)
            
            start_time = time.time()
            execution_data = []
            
            for i, target_point in enumerate(trajectory):
                try:
                    # 获取当前状态
                    current_state = self.rov.get_rov_state()
                    if not current_state:
                        continue
                    
                    # 准备目标状态
                    target_dict = {
                        'x': target_point.x,
                        'y': target_point.y,
                        'z': target_point.z,
                        'yaw': target_point.yaw
                    }
                    
                    current_dict = {
                        'x': current_state.position[0],
                        'y': current_state.position[1],
                        'z': current_state.position[2],
                        'yaw': current_state.orientation[2]
                    }
                    
                    # 计算控制输出
                    control_output = self.control_system.execute_control(current_dict, target_dict)
                    
                    # 转换为推进器控制
                    pwm_values = self.rov.wrench_to_pwm(control_output)
                    success = self.rov.set_thruster_pwm(pwm_values.tolist(), 0.1)
                    
                    if not success:
                        print(f"⚠️ 第{i+1}个轨迹点控制失败")
                    
                    # 记录执行数据
                    execution_point = {
                        'time': time.time() - start_time,
                        'target': [target_point.x, target_point.y, target_point.z, target_point.yaw],
                        'current': current_state.position.tolist() + [current_state.orientation[2]],
                        'control': control_output.tolist(),
                        'error': np.linalg.norm(np.array([target_point.x, target_point.y, target_point.z]) - 
                                              current_state.position)
                    }
                    execution_data.append(execution_point)
                    
                    # 进度显示 (每10%显示一次)
                    if (i + 1) % max(1, len(trajectory) // 10) == 0:
                        progress = (i + 1) / len(trajectory) * 100
                        elapsed = time.time() - start_time
                        error = execution_point['error']
                        print(f"  📈 进度: {progress:.0f}% | 用时: {elapsed:.1f}s | 误差: {error:.3f}m")
                    
                    time.sleep(0.1)  # 控制周期
                    
                except KeyboardInterrupt:
                    print("\n⏹️ 用户中断轨迹执行")
                    break
                except Exception as point_error:
                    print(f"⚠️ 轨迹点{i+1}执行错误: {point_error}")
                    continue
            
            # 停止推进器
            self.rov.emergency_stop()
            
            # 轨迹执行统计
            total_time = time.time() - start_time
            if execution_data:
                errors = [point['error'] for point in execution_data]
                avg_error = np.mean(errors)
                max_error = np.max(errors)
                final_error = errors[-1] if errors else 0
                
                print(f"\n📊 {trajectory_name}执行统计:")
                print(f"   执行时间: {total_time:.1f}秒")
                print(f"   平均误差: {avg_error:.3f}m")
                print(f"   最大误差: {max_error:.3f}m") 
                print(f"   最终误差: {final_error:.3f}m")
                print(f"   成功率: {len(execution_data)}/{len(trajectory)} ({len(execution_data)/len(trajectory)*100:.1f}%)")
            
            # 保存执行数据
            self.save_trajectory_execution(execution_data, trajectory_name)
            
            print(f"✅ {trajectory_name}轨迹执行完成")
            
        except Exception as e:
            print(f"❌ {trajectory_name}轨迹执行失败: {e}")
            self.rov.emergency_stop()
    
    def visualize_trajectory(self, trajectory: List[TrajectoryPoint], 
                           title: str, save_path: str = None):
        """可视化轨迹"""
        try:
            if not trajectory:
                return
            
            # 提取轨迹数据
            x = [p.x for p in trajectory]
            y = [p.y for p in trajectory]  
            z = [p.z for p in trajectory]
            
            # 创建3D图形
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')
            
            # 绘制轨迹
            ax.plot(x, y, z, 'b-', linewidth=2, label='轨迹路径')
            ax.scatter(x[0], y[0], z[0], color='green', s=100, label='起点')
            ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='终点')
            
            # 绘制方向箭头 (每10个点绘制一个)
            for i in range(0, len(trajectory), max(1, len(trajectory)//20)):
                p = trajectory[i]
                arrow_length = 0.3
                dx = arrow_length * math.cos(p.yaw)
                dy = arrow_length * math.sin(p.yaw)
                ax.quiver(p.x, p.y, p.z, dx, dy, 0, 
                         color='orange', alpha=0.7, length=0.1)
            
            # 设置图形属性
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'{title}\n轨迹点数: {len(trajectory)}')
            ax.legend()
            
            # 设置相等的坐标轴比例
            max_range = max(max(x) - min(x), max(y) - min(y), max(z) - min(z)) / 2
            mid_x, mid_y, mid_z = (max(x) + min(x)) / 2, (max(y) + min(y)) / 2, (max(z) + min(z)) / 2
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            
            plt.tight_layout()
            
            if save_path:
                plt.savefig(save_path, dpi=150, bbox_inches='tight')
                print(f"💾 轨迹图已保存: {save_path}")
            
            plt.show()
            
        except Exception as e:
            print(f"❌ 轨迹可视化失败: {e}")
    
    def save_trajectory_execution(self, execution_data: list, trajectory_name: str):
        """保存轨迹执行数据"""
        try:
            timestamp = int(time.time())
            filename = f"trajectory_{trajectory_name.replace(' ', '_')}_{timestamp}.json"
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump({
                    'trajectory_name': trajectory_name,
                    'timestamp': timestamp,
                    'execution_data': execution_data,
                    'statistics': {
                        'total_points': len(execution_data),
                        'duration': execution_data[-1]['time'] if execution_data else 0,
                        'average_error': np.mean([p['error'] for p in execution_data]) if execution_data else 0
                    }
                }, f, indent=2)
            
            print(f"💾 轨迹数据已保存: {filename}")
            
        except Exception as e:
            print(f"⚠️ 保存轨迹数据失败: {e}")
    
    def run_demo(self, demo_name: str = 'all'):
        """运行轨迹演示"""
        print("🎯 HAAV_Sim 高级轨迹演示程序")
        print("=" * 50)
        
        # 初始化系统
        if not self.initialize():
            return False
        
        try:
            self.demo_running = True
            
            if demo_name == 'all':
                print("🎬 运行所有轨迹演示...")
                for name, config in self.demos.items():
                    if name != 'complex_mission':  # 复合任务单独处理
                        print(f"\n{'='*60}")
                        print(f"🎯 {config['name']}")
                        print(f"📝 {config['description']}")
                        print(f"⏱️ 预计时长: {config['duration']}秒")
                        
                        config['func']()
                        
                        print("\n⏸️ 演示间隔暂停...")
                        time.sleep(3)
                
                # 最后运行复合任务
                print(f"\n{'='*60}")
                print("🏆 最终挑战: 复合轨迹任务")
                self.demos['complex_mission']['func']()
                
            elif demo_name in self.demos:
                config = self.demos[demo_name]
                print(f"🎯 运行单个演示: {config['name']}")
                print(f"📝 {config['description']}")
                config['func']()
                
            elif demo_name == 'preview':
                print("👀 轨迹预览模式 (仅生成和显示，不执行)")
                self.preview_all_trajectories()
                
            else:
                print(f"❌ 未知演示: {demo_name}")
                print(f"💡 可用演示: {list(self.demos.keys())}")
                return False
            
            print("\n🎉 轨迹演示程序完成!")
            return True
            
        except KeyboardInterrupt:
            print("\n⏹️ 用户中断演示")
            return False
        
        finally:
            self.demo_running = False
            if self.rov:
                self.rov.shutdown()
    
    def preview_all_trajectories(self):
        """预览所有轨迹 (不执行，仅可视化)"""
        print("👀 生成所有轨迹预览...")
        
        center = np.array([0, 0, -2])  # 假设中心位置
        
        trajectories = {
            '螺旋上升': self.trajectory_generator.generate_spiral_ascent(
                center, 0.5, 2.0, -3, 0, 3.0, 60.0),
            '八字轨迹': self.trajectory_generator.generate_figure_eight(
                center, 3.0, 2.0, -2, 45.0),
            '正弦波': self.trajectory_generator.generate_sine_wave(
                center, center + np.array([8, 2, -1]), 1.5, 2.0, 40.0),
            '3D螺旋': self.trajectory_generator.generate_helix_3d(
                center, 2.0, 1.0, 4.0, 50.0),
            '3D双纽线': self.trajectory_generator.generate_lemniscate_3d(
                center, 2.5, -2, 55.0),
            '玫瑰线': self.trajectory_generator.generate_rose_curve(
                center, 2.0, 5, -2, 45.0),
            '四叶草': self.trajectory_generator.generate_cloverleaf(
                center, 2.0, -2, 40.0)
        }
        
        for name, trajectory in trajectories.items():
            if trajectory:
                print(f"🎨 绘制{name}轨迹...")
                self.visualize_trajectory(trajectory, name, f"{name}_preview.png")
        
        print("✅ 所有轨迹预览完成!")

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='HAAV_Sim高级轨迹演示程序')
    parser.add_argument('--demo', '-d',
                       choices=list(TrajectoryDemoRunner().demos.keys()) + ['all', 'preview'],
                       default='all',
                       help='选择要运行的轨迹演示')
    parser.add_argument('--visualize', '-v', action='store_true',
                       help='启用轨迹可视化')
    
    args = parser.parse_args()
    
    # 创建并运行演示
    demo_runner = TrajectoryDemoRunner()
    success = demo_runner.run_demo(args.demo)
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())