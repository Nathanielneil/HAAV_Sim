#!/usr/bin/env python3
"""
高级任务演示程序
包含多种实用的ROV任务场景：搜索救援、水下检测、科学采样等
"""

import sys
import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import json
import random
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum
import threading
from concurrent.futures import ThreadPoolExecutor

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'python'))

from rov_client import ROVController
from control_algorithms import ROVControlSystem, ControlMode, ObstacleAvoidance
from trajectory_demos import TrajectoryGenerator, TrajectoryPoint

class MissionType(Enum):
    """任务类型"""
    SEARCH_RESCUE = "search_rescue"
    UNDERWATER_INSPECTION = "underwater_inspection"
    SCIENTIFIC_SAMPLING = "scientific_sampling"
    PIPELINE_FOLLOWING = "pipeline_following"
    STRUCTURE_MAPPING = "structure_mapping"
    ENVIRONMENTAL_MONITORING = "environmental_monitoring"
    DEBRIS_REMOVAL = "debris_removal"
    SURVEILLANCE_PATROL = "surveillance_patrol"

@dataclass
class MissionWaypoint:
    """任务航点"""
    position: np.ndarray
    action: str  # hover, sample, inspect, photo, scan
    duration: float
    parameters: Dict = None

@dataclass  
class MissionObjective:
    """任务目标"""
    id: str
    position: np.ndarray
    type: str  # target, obstacle, sample_point, inspection_point
    status: str = "pending"  # pending, completed, failed
    data: Dict = None

class AdvancedMissionController:
    """高级任务控制器"""
    
    def __init__(self):
        self.rov_controller = None
        self.control_system = ROVControlSystem()
        self.trajectory_generator = TrajectoryGenerator()
        
        # 任务状态
        self.current_mission = None
        self.mission_objectives = []
        self.completed_objectives = []
        self.mission_data = []
        self.mission_running = False
        
        # 传感器数据
        self.sensor_data_log = []
        self.image_captures = []
        
        # 任务参数
        self.mission_params = {
            'search_pattern_spacing': 2.0,  # 搜索网格间距
            'inspection_distance': 1.0,     # 检查距离
            'sampling_time': 10.0,          # 采样时间
            'photo_interval': 5.0,          # 拍照间隔
            'patrol_speed': 1.0,            # 巡逻速度
            'safety_margin': 0.5            # 安全边距
        }
    
    def initialize_rov(self) -> bool:
        """初始化ROV"""
        try:
            print("🚁 初始化ROV系统...")
            self.rov_controller = ROVController()
            
            if self.rov_controller.initialize():
                print("✅ ROV系统初始化成功")
                return True
            else:
                print("❌ ROV初始化失败")
                return False
        except Exception as e:
            print(f"❌ ROV初始化异常: {e}")
            return False
    
    def mission_search_rescue(self, search_area: Tuple[np.ndarray, np.ndarray], 
                             target_signatures: List[str] = None):
        """搜索救援任务"""
        print("\n🔍 搜索救援任务")
        print("=" * 40)
        
        try:
            area_min, area_max = search_area
            area_size = area_max - area_min
            
            # 生成搜索网格
            spacing = self.mission_params['search_pattern_spacing']
            grid_points = self.generate_search_grid(area_min, area_max, spacing)
            
            print(f"🗺️ 搜索区域: {area_size[0]:.1f}m × {area_size[1]:.1f}m")
            print(f"📍 搜索点数: {len(grid_points)}")
            print(f"⚡ 预计时长: {len(grid_points) * 15 / 60:.1f}分钟")
            
            mission_start_time = time.time()
            detected_targets = []
            
            # 执行搜索模式
            for i, search_point in enumerate(grid_points):
                print(f"\n📍 搜索点 {i+1}/{len(grid_points)}: ({search_point[0]:.1f}, {search_point[1]:.1f})")
                
                # 移动到搜索点
                success = self.move_to_position_with_monitoring(search_point, hover_time=3.0)
                
                if success:
                    # 执行搜索动作
                    search_result = self.execute_search_action(search_point, target_signatures)
                    
                    if search_result['targets_detected']:
                        detected_targets.extend(search_result['targets'])
                        print(f"🎯 发现目标: {len(search_result['targets'])}个")
                        
                        # 详细检查检测到的目标
                        for target in search_result['targets']:
                            self.detailed_target_inspection(target)
                
                # 检查任务中断
                if self.check_mission_interrupt():
                    break
            
            # 任务总结
            mission_duration = time.time() - mission_start_time
            print(f"\n📊 搜索救援任务完成")
            print(f"   搜索时间: {mission_duration/60:.1f}分钟")
            print(f"   搜索点数: {len(grid_points)}")
            print(f"   发现目标: {len(detected_targets)}个")
            
            if detected_targets:
                print(f"\n🎯 发现的目标:")
                for i, target in enumerate(detected_targets):
                    print(f"   目标{i+1}: 位置({target['position'][0]:.1f}, {target['position'][1]:.1f})")
                    print(f"           置信度: {target['confidence']:.2f}")
                    print(f"           类型: {target['type']}")
            
            # 保存搜索数据
            self.save_mission_data({
                'mission_type': 'search_rescue',
                'duration': mission_duration,
                'search_area': [area_min.tolist(), area_max.tolist()],
                'targets_detected': len(detected_targets),
                'targets': detected_targets,
                'sensor_data': self.sensor_data_log[-len(grid_points):] if self.sensor_data_log else []
            }, "search_rescue_mission")
            
            return len(detected_targets) > 0
            
        except Exception as e:
            print(f"❌ 搜索救援任务失败: {e}")
            return False
    
    def mission_underwater_inspection(self, inspection_targets: List[Dict]):
        """水下检查任务"""
        print("\n🔍 水下结构检查任务")
        print("=" * 40)
        
        try:
            print(f"🏗️ 检查目标数: {len(inspection_targets)}")
            
            inspection_results = []
            mission_start_time = time.time()
            
            for i, target in enumerate(inspection_targets):
                print(f"\n🔍 检查目标 {i+1}/{len(inspection_targets)}: {target['name']}")
                
                target_position = np.array(target['position'])
                inspection_type = target.get('type', 'visual')
                
                # 围绕目标进行多角度检查
                inspection_result = self.execute_multi_angle_inspection(
                    target_position, inspection_type, target.get('radius', 2.0)
                )
                
                inspection_results.append({
                    'target': target,
                    'result': inspection_result,
                    'timestamp': time.time()
                })
                
                # 生成检查报告
                self.generate_inspection_report(target, inspection_result)
            
            # 任务总结
            mission_duration = time.time() - mission_start_time
            passed_inspections = sum(1 for r in inspection_results if r['result']['status'] == 'passed')
            
            print(f"\n📊 水下检查任务完成")
            print(f"   检查时间: {mission_duration/60:.1f}分钟")
            print(f"   检查目标: {len(inspection_targets)}个")
            print(f"   通过检查: {passed_inspections}个")
            print(f"   问题发现: {len(inspection_targets) - passed_inspections}个")
            
            # 保存检查数据
            self.save_mission_data({
                'mission_type': 'underwater_inspection',
                'duration': mission_duration,
                'targets_inspected': len(inspection_targets),
                'passed_inspections': passed_inspections,
                'inspection_results': inspection_results
            }, "underwater_inspection_mission")
            
            return True
            
        except Exception as e:
            print(f"❌ 水下检查任务失败: {e}")
            return False
    
    def mission_scientific_sampling(self, sampling_locations: List[Dict]):
        """科学采样任务"""
        print("\n🧪 科学采样任务")
        print("=" * 40)
        
        try:
            print(f"🎯 采样位置数: {len(sampling_locations)}")
            
            sampling_results = []
            mission_start_time = time.time()
            
            for i, location in enumerate(sampling_locations):
                print(f"\n🧪 采样点 {i+1}/{len(sampling_locations)}: {location['name']}")
                
                sampling_position = np.array(location['position'])
                sampling_type = location.get('type', 'water')
                sampling_duration = location.get('duration', self.mission_params['sampling_time'])
                
                # 移动到采样位置
                success = self.move_to_position_with_monitoring(sampling_position)
                
                if success:
                    # 执行采样动作
                    sampling_result = self.execute_sampling_action(
                        sampling_position, sampling_type, sampling_duration
                    )
                    
                    sampling_results.append({
                        'location': location,
                        'result': sampling_result,
                        'timestamp': time.time()
                    })
                    
                    print(f"✅ 采样完成: {sampling_result['sample_id']}")
                    print(f"   采样量: {sampling_result['volume']:.1f}ml")
                    print(f"   质量评估: {sampling_result['quality']}")
            
            # 任务总结
            mission_duration = time.time() - mission_start_time
            successful_samples = sum(1 for r in sampling_results if r['result']['status'] == 'success')
            
            print(f"\n📊 科学采样任务完成")
            print(f"   采样时间: {mission_duration/60:.1f}分钟")
            print(f"   采样点数: {len(sampling_locations)}")
            print(f"   成功采样: {successful_samples}个")
            print(f"   样品总量: {sum(r['result']['volume'] for r in sampling_results):.1f}ml")
            
            # 保存采样数据
            self.save_mission_data({
                'mission_type': 'scientific_sampling',
                'duration': mission_duration,
                'locations_sampled': len(sampling_locations),
                'successful_samples': successful_samples,
                'sampling_results': sampling_results,
                'total_volume': sum(r['result']['volume'] for r in sampling_results)
            }, "scientific_sampling_mission")
            
            return successful_samples > 0
            
        except Exception as e:
            print(f"❌ 科学采样任务失败: {e}")
            return False
    
    def mission_pipeline_following(self, pipeline_start: np.ndarray, 
                                  pipeline_direction: float, pipeline_length: float):
        """管道跟踪任务"""
        print("\n🚰 管道跟踪检查任务")  
        print("=" * 40)
        
        try:
            print(f"📍 起点: ({pipeline_start[0]:.1f}, {pipeline_start[1]:.1f})")
            print(f"🧭 方向: {math.degrees(pipeline_direction):.1f}°")
            print(f"📏 长度: {pipeline_length:.1f}m")
            
            # 生成管道跟踪轨迹
            pipeline_points = self.generate_pipeline_following_trajectory(
                pipeline_start, pipeline_direction, pipeline_length
            )
            
            inspection_findings = []
            mission_start_time = time.time()
            
            print(f"🛤️ 跟踪点数: {len(pipeline_points)}")
            
            for i, point in enumerate(pipeline_points):
                print(f"\n📍 跟踪点 {i+1}/{len(pipeline_points)}")
                
                # 移动到跟踪点
                success = self.move_to_position_with_monitoring(point['position'])
                
                if success:
                    # 执行管道检查
                    inspection_result = self.execute_pipeline_inspection(point)
                    
                    if inspection_result['anomalies_detected']:
                        inspection_findings.extend(inspection_result['anomalies'])
                        print(f"⚠️ 发现异常: {len(inspection_result['anomalies'])}个")
                
                # 进度显示
                progress = (i + 1) / len(pipeline_points) * 100
                print(f"📊 跟踪进度: {progress:.1f}%")
            
            # 任务总结
            mission_duration = time.time() - mission_start_time
            
            print(f"\n📊 管道跟踪任务完成")
            print(f"   跟踪时间: {mission_duration/60:.1f}分钟")
            print(f"   跟踪长度: {pipeline_length:.1f}m")
            print(f"   检查点数: {len(pipeline_points)}")
            print(f"   发现异常: {len(inspection_findings)}个")
            
            if inspection_findings:
                print(f"\n⚠️ 发现的异常:")
                for i, finding in enumerate(inspection_findings):
                    print(f"   异常{i+1}: {finding['type']} at ({finding['position'][0]:.1f}, {finding['position'][1]:.1f})")
                    print(f"           严重程度: {finding['severity']}")
            
            # 保存跟踪数据
            self.save_mission_data({
                'mission_type': 'pipeline_following',
                'duration': mission_duration,
                'pipeline_length': pipeline_length,
                'inspection_points': len(pipeline_points),
                'anomalies_found': len(inspection_findings),
                'findings': inspection_findings
            }, "pipeline_following_mission")
            
            return True
            
        except Exception as e:
            print(f"❌ 管道跟踪任务失败: {e}")
            return False
    
    def mission_environmental_monitoring(self, monitoring_stations: List[Dict], 
                                       monitoring_duration: float = 60.0):
        """环境监控任务"""
        print("\n🌊 环境监控任务")
        print("=" * 40)
        
        try:
            print(f"📊 监控站点: {len(monitoring_stations)}")
            print(f"⏱️ 监控时长: {monitoring_duration/60:.1f}分钟")
            
            environmental_data = []
            mission_start_time = time.time()
            
            for i, station in enumerate(monitoring_stations):
                print(f"\n📊 监控站点 {i+1}/{len(monitoring_stations)}: {station['name']}")
                
                station_position = np.array(station['position'])
                parameters = station.get('parameters', ['temperature', 'turbidity', 'ph', 'oxygen'])
                
                # 移动到监控位置
                success = self.move_to_position_with_monitoring(station_position)
                
                if success:
                    # 执行环境监控
                    monitoring_result = self.execute_environmental_monitoring(
                        station_position, parameters, monitoring_duration / len(monitoring_stations)
                    )
                    
                    environmental_data.append({
                        'station': station,
                        'data': monitoring_result,
                        'timestamp': time.time()
                    })
                    
                    # 显示监控结果
                    print(f"🌡️ 温度: {monitoring_result.get('temperature', 'N/A')}°C")
                    print(f"💧 浊度: {monitoring_result.get('turbidity', 'N/A')} NTU")
                    print(f"⚗️ pH值: {monitoring_result.get('ph', 'N/A')}")
                    print(f"🫧 溶氧: {monitoring_result.get('oxygen', 'N/A')} mg/L")
            
            # 数据分析
            analysis_result = self.analyze_environmental_data(environmental_data)
            
            # 任务总结
            mission_duration = time.time() - mission_start_time
            
            print(f"\n📊 环境监控任务完成")
            print(f"   监控时间: {mission_duration/60:.1f}分钟")
            print(f"   监控站点: {len(monitoring_stations)}个")
            print(f"   数据质量: {analysis_result['data_quality']:.1f}%")
            print(f"   异常指标: {len(analysis_result['anomalies'])}个")
            
            # 环境评估
            print(f"\n🌊 环境评估结果:")
            print(f"   整体状态: {analysis_result['overall_status']}")
            print(f"   平均温度: {analysis_result['avg_temperature']:.1f}°C")
            print(f"   平均浊度: {analysis_result['avg_turbidity']:.1f} NTU")
            
            # 保存监控数据
            self.save_mission_data({
                'mission_type': 'environmental_monitoring',
                'duration': mission_duration,
                'stations_monitored': len(monitoring_stations),
                'environmental_data': environmental_data,
                'analysis_result': analysis_result
            }, "environmental_monitoring_mission")
            
            return True
            
        except Exception as e:
            print(f"❌ 环境监控任务失败: {e}")
            return False
    
    def mission_surveillance_patrol(self, patrol_route: List[np.ndarray], 
                                   patrol_cycles: int = 3):
        """监视巡逻任务"""
        print("\n👁️ 监视巡逻任务")
        print("=" * 40)
        
        try:
            print(f"🛤️ 巡逻路线: {len(patrol_route)}个点")
            print(f"🔄 巡逻循环: {patrol_cycles}次")
            
            surveillance_events = []
            mission_start_time = time.time()
            
            for cycle in range(patrol_cycles):
                print(f"\n🔄 巡逻循环 {cycle+1}/{patrol_cycles}")
                
                for i, waypoint in enumerate(patrol_route):
                    print(f"📍 巡逻点 {i+1}/{len(patrol_route)}")
                    
                    # 移动到巡逻点
                    success = self.move_to_position_with_monitoring(waypoint, 
                                                                 hover_time=5.0)
                    
                    if success:
                        # 执行监视行为
                        surveillance_result = self.execute_surveillance_scan(waypoint)
                        
                        if surveillance_result['events_detected']:
                            surveillance_events.extend(surveillance_result['events'])
                            print(f"⚠️ 检测到事件: {len(surveillance_result['events'])}个")
                        
                        # 拍摄监控照片
                        self.capture_surveillance_image(waypoint, f"patrol_c{cycle+1}_p{i+1}")
                
                # 循环间暂停
                if cycle < patrol_cycles - 1:
                    print("⏸️ 循环间暂停...")
                    time.sleep(10)
            
            # 任务总结
            mission_duration = time.time() - mission_start_time
            
            print(f"\n📊 监视巡逻任务完成")
            print(f"   巡逻时间: {mission_duration/60:.1f}分钟")
            print(f"   巡逻循环: {patrol_cycles}次")
            print(f"   巡逻点数: {len(patrol_route) * patrol_cycles}")
            print(f"   检测事件: {len(surveillance_events)}个")
            print(f"   拍摄照片: {len(self.image_captures)}张")
            
            if surveillance_events:
                print(f"\n⚠️ 检测到的事件:")
                for i, event in enumerate(surveillance_events):
                    print(f"   事件{i+1}: {event['type']} at ({event['position'][0]:.1f}, {event['position'][1]:.1f})")
                    print(f"           时间: {time.strftime('%H:%M:%S', time.localtime(event['timestamp']))}")
            
            # 保存巡逻数据
            self.save_mission_data({
                'mission_type': 'surveillance_patrol',
                'duration': mission_duration,
                'patrol_cycles': patrol_cycles,
                'patrol_points': len(patrol_route) * patrol_cycles,
                'events_detected': len(surveillance_events),
                'events': surveillance_events,
                'images_captured': len(self.image_captures)
            }, "surveillance_patrol_mission")
            
            return True
            
        except Exception as e:
            print(f"❌ 监视巡逻任务失败: {e}")
            return False
    
    # 辅助方法实现
    
    def generate_search_grid(self, area_min: np.ndarray, area_max: np.ndarray, 
                           spacing: float) -> List[np.ndarray]:
        """生成搜索网格"""
        grid_points = []
        
        x_range = np.arange(area_min[0], area_max[0] + spacing, spacing)
        y_range = np.arange(area_min[1], area_max[1] + spacing, spacing)
        
        # 生成蛇形搜索模式
        for i, y in enumerate(y_range):
            if i % 2 == 0:  # 偶数行从左到右
                x_values = x_range
            else:  # 奇数行从右到左
                x_values = x_range[::-1]
            
            for x in x_values:
                # 搜索深度在区域平均值附近
                z = (area_min[2] + area_max[2]) / 2
                grid_points.append(np.array([x, y, z]))
        
        return grid_points
    
    def move_to_position_with_monitoring(self, target_position: np.ndarray, 
                                       hover_time: float = 2.0) -> bool:
        """带监控的位置移动"""
        try:
            # 使用位置控制移动
            success = self.rov_controller.move_to_position(
                target_position[0], target_position[1], target_position[2],
                target_yaw=0.0, timeout=30.0
            )
            
            if success and hover_time > 0:
                # 悬停指定时间
                time.sleep(hover_time)
                
                # 记录位置数据
                current_state = self.rov_controller.get_rov_state()
                if current_state:
                    self.sensor_data_log.append({
                        'timestamp': time.time(),
                        'target_position': target_position.tolist(),
                        'actual_position': current_state.position.tolist(),
                        'depth': current_state.depth
                    })
            
            return success
            
        except Exception as e:
            print(f"⚠️ 位置移动失败: {e}")
            return False
    
    def execute_search_action(self, position: np.ndarray, 
                            target_signatures: List[str] = None) -> Dict:
        """执行搜索动作"""
        result = {
            'position': position.tolist(),
            'timestamp': time.time(),
            'targets_detected': False,
            'targets': []
        }
        
        try:
            # 获取相机图像
            rgb_image = self.rov_controller.get_camera_image()
            depth_image = self.rov_controller.get_depth_image()
            
            # 模拟目标检测算法
            if rgb_image is not None:
                # 简单的目标检测模拟
                detection_probability = random.random()
                
                if detection_probability > 0.7:  # 70%概率不检测到目标
                    num_targets = random.randint(1, 3)
                    
                    for i in range(num_targets):
                        target = {
                            'id': f"target_{int(time.time())}_{i}",
                            'position': position + np.random.normal(0, 1, 3),
                            'confidence': random.uniform(0.6, 0.95),
                            'type': random.choice(['debris', 'object', 'anomaly']),
                            'size': random.uniform(0.1, 2.0)
                        }
                        result['targets'].append(target)
                    
                    result['targets_detected'] = True
            
            # 保存搜索数据
            result['sensor_data'] = {
                'rgb_available': rgb_image is not None,
                'depth_available': depth_image is not None,
                'image_quality': random.uniform(0.7, 1.0)
            }
            
        except Exception as e:
            print(f"⚠️ 搜索动作执行失败: {e}")
        
        return result
    
    def detailed_target_inspection(self, target: Dict):
        """详细目标检查"""
        print(f"🔍 详细检查目标: {target['id']}")
        
        target_position = np.array(target['position'])
        
        # 围绕目标进行近距离检查
        inspection_angles = [0, math.pi/2, math.pi, 3*math.pi/2]
        inspection_distance = 0.8
        
        for angle in inspection_angles:
            offset_x = inspection_distance * math.cos(angle)
            offset_y = inspection_distance * math.sin(angle)
            
            inspection_pos = target_position + np.array([offset_x, offset_y, 0])
            
            success = self.move_to_position_with_monitoring(inspection_pos, hover_time=2.0)
            if success:
                # 拍摄详细照片
                self.capture_target_image(target, angle)
        
        print(f"✅ 目标 {target['id']} 详细检查完成")
    
    def execute_multi_angle_inspection(self, target_position: np.ndarray, 
                                     inspection_type: str, radius: float) -> Dict:
        """多角度检查执行"""
        result = {
            'status': 'passed',
            'issues_found': [],
            'inspection_angles': [],
            'quality_score': 0.0
        }
        
        try:
            # 生成检查角度
            if inspection_type == 'comprehensive':
                angles = [i * math.pi / 4 for i in range(8)]  # 8个角度
            else:
                angles = [0, math.pi/2, math.pi, 3*math.pi/2]  # 4个角度
            
            inspection_scores = []
            
            for angle in angles:
                # 计算检查位置
                offset_x = radius * math.cos(angle)
                offset_y = radius * math.sin(angle)
                inspection_pos = target_position + np.array([offset_x, offset_y, 0])
                
                # 移动到检查位置
                success = self.move_to_position_with_monitoring(inspection_pos, hover_time=3.0)
                
                if success:
                    # 执行检查
                    angle_result = self.perform_angle_inspection(target_position, angle)
                    result['inspection_angles'].append({
                        'angle': math.degrees(angle),
                        'score': angle_result['score'],
                        'issues': angle_result['issues']
                    })
                    
                    inspection_scores.append(angle_result['score'])
                    
                    if angle_result['issues']:
                        result['issues_found'].extend(angle_result['issues'])
            
            # 计算总体质量评分
            if inspection_scores:
                result['quality_score'] = np.mean(inspection_scores)
                if result['quality_score'] < 0.7 or result['issues_found']:
                    result['status'] = 'failed'
            
        except Exception as e:
            print(f"⚠️ 多角度检查失败: {e}")
            result['status'] = 'error'
        
        return result
    
    def perform_angle_inspection(self, target_position: np.ndarray, angle: float) -> Dict:
        """执行单角度检查"""
        result = {
            'score': random.uniform(0.6, 1.0),  # 模拟检查评分
            'issues': []
        }
        
        # 模拟缺陷检测
        if random.random() > 0.8:  # 20%概率发现问题
            issue_types = ['crack', 'corrosion', 'deformation', 'obstruction']
            result['issues'].append({
                'type': random.choice(issue_types),
                'severity': random.choice(['low', 'medium', 'high']),
                'position': target_position.tolist(),
                'angle': math.degrees(angle)
            })
            result['score'] *= 0.7  # 降低评分
        
        return result
    
    def execute_sampling_action(self, position: np.ndarray, 
                              sampling_type: str, duration: float) -> Dict:
        """执行采样动作"""
        result = {
            'status': 'success',
            'sample_id': f"sample_{int(time.time())}",
            'type': sampling_type,
            'volume': 0.0,
            'quality': 'good'
        }
        
        try:
            print(f"🧪 开始采样 ({duration}秒)...")
            
            # 模拟采样过程
            for i in range(int(duration)):
                print(f"   采样进度: {(i+1)/duration*100:.0f}%")
                time.sleep(1)
            
            # 模拟采样结果
            if sampling_type == 'water':
                result['volume'] = random.uniform(50, 200)  # ml
            elif sampling_type == 'sediment':
                result['volume'] = random.uniform(10, 50)   # ml
            else:
                result['volume'] = random.uniform(5, 100)   # ml
            
            # 质量评估
            quality_factors = ['excellent', 'good', 'fair', 'poor']
            result['quality'] = random.choice(quality_factors)
            
            # 环境参数记录
            result['environmental_conditions'] = {
                'temperature': random.uniform(8, 15),  # °C
                'pressure': random.uniform(1.5, 3.0),  # atm
                'turbidity': random.uniform(1, 10),    # NTU
                'flow_rate': random.uniform(0, 0.5)    # m/s
            }
            
            print(f"✅ 采样完成: {result['volume']:.1f}ml, 质量: {result['quality']}")
            
        except Exception as e:
            print(f"⚠️ 采样失败: {e}")
            result['status'] = 'failed'
        
        return result
    
    def generate_pipeline_following_trajectory(self, start: np.ndarray, 
                                             direction: float, length: float) -> List[Dict]:
        """生成管道跟踪轨迹"""
        points = []
        
        # 计算跟踪间距
        spacing = 3.0  # 每3米一个检查点
        num_points = int(length / spacing) + 1
        
        for i in range(num_points):
            distance = i * spacing
            
            # 计算位置
            x = start[0] + distance * math.cos(direction)
            y = start[1] + distance * math.sin(direction)  
            z = start[2]
            
            point = {
                'position': np.array([x, y, z]),
                'distance': distance,
                'section_id': f"section_{i}"
            }
            points.append(point)
        
        return points
    
    def execute_pipeline_inspection(self, point: Dict) -> Dict:
        """执行管道检查"""
        result = {
            'section_id': point['section_id'],
            'position': point['position'].tolist(),
            'anomalies_detected': False,
            'anomalies': [],
            'condition_score': random.uniform(0.7, 1.0)
        }
        
        # 模拟异常检测
        if random.random() > 0.85:  # 15%概率发现异常
            anomaly_types = ['leak', 'corrosion', 'blockage', 'damage']
            severity_levels = ['minor', 'moderate', 'severe']
            
            anomaly = {
                'type': random.choice(anomaly_types),
                'severity': random.choice(severity_levels),
                'position': point['position'].tolist(),
                'confidence': random.uniform(0.6, 0.95)
            }
            
            result['anomalies'].append(anomaly)
            result['anomalies_detected'] = True
            result['condition_score'] *= 0.5  # 降低管道状况评分
        
        return result
    
    def execute_environmental_monitoring(self, position: np.ndarray, 
                                       parameters: List[str], duration: float) -> Dict:
        """执行环境监控"""
        result = {}
        
        try:
            print(f"📊 环境监控 ({duration}秒)...")
            
            # 模拟传感器读数
            for param in parameters:
                if param == 'temperature':
                    result['temperature'] = random.uniform(8, 15)  # °C
                elif param == 'turbidity':
                    result['turbidity'] = random.uniform(1, 20)    # NTU
                elif param == 'ph':
                    result['ph'] = random.uniform(7.5, 8.5)
                elif param == 'oxygen':
                    result['oxygen'] = random.uniform(6, 12)       # mg/L
                elif param == 'salinity':
                    result['salinity'] = random.uniform(30, 35)    # ppt
                elif param == 'pressure':
                    result['pressure'] = random.uniform(1.5, 3.0) # atm
            
            # 监控时间内的数据变化
            time_series_data = {}
            sampling_interval = max(1, duration / 10)
            
            for i in range(10):
                timestamp = time.time() + i * sampling_interval
                time_series_data[timestamp] = {
                    param: result[param] + random.normal(0, result[param] * 0.02)
                    for param in parameters if param in result
                }
                time.sleep(sampling_interval / 10)  # 压缩时间模拟
            
            result['time_series'] = time_series_data
            result['monitoring_duration'] = duration
            result['data_quality'] = random.uniform(0.85, 1.0)
            
        except Exception as e:
            print(f"⚠️ 环境监控失败: {e}")
        
        return result
    
    def analyze_environmental_data(self, environmental_data: List[Dict]) -> Dict:
        """分析环境数据"""
        analysis = {
            'overall_status': 'normal',
            'anomalies': [],
            'data_quality': 0.0,
            'avg_temperature': 0.0,
            'avg_turbidity': 0.0,
            'trends': {}
        }
        
        try:
            if not environmental_data:
                return analysis
            
            # 计算平均值
            temp_values = []
            turbidity_values = []
            quality_scores = []
            
            for entry in environmental_data:
                data = entry['data']
                if 'temperature' in data:
                    temp_values.append(data['temperature'])
                if 'turbidity' in data:
                    turbidity_values.append(data['turbidity'])
                if 'data_quality' in data:
                    quality_scores.append(data['data_quality'])
            
            if temp_values:
                analysis['avg_temperature'] = np.mean(temp_values)
            if turbidity_values:
                analysis['avg_turbidity'] = np.mean(turbidity_values)
            if quality_scores:
                analysis['data_quality'] = np.mean(quality_scores) * 100
            
            # 异常检测
            for entry in environmental_data:
                data = entry['data']
                station_name = entry['station']['name']
                
                # 检查异常值
                if 'temperature' in data and (data['temperature'] < 5 or data['temperature'] > 20):
                    analysis['anomalies'].append({
                        'station': station_name,
                        'parameter': 'temperature',
                        'value': data['temperature'],
                        'threshold': '5-20°C'
                    })
                
                if 'turbidity' in data and data['turbidity'] > 15:
                    analysis['anomalies'].append({
                        'station': station_name,
                        'parameter': 'turbidity', 
                        'value': data['turbidity'],
                        'threshold': '<15 NTU'
                    })
            
            # 状态评估
            if len(analysis['anomalies']) > 0:
                analysis['overall_status'] = 'warning' if len(analysis['anomalies']) < 3 else 'critical'
            
        except Exception as e:
            print(f"⚠️ 环境数据分析失败: {e}")
        
        return analysis
    
    def execute_surveillance_scan(self, position: np.ndarray) -> Dict:
        """执行监视扫描"""
        result = {
            'position': position.tolist(),
            'timestamp': time.time(),
            'events_detected': False,
            'events': []
        }
        
        try:
            # 360度扫描
            scan_angles = [i * math.pi / 4 for i in range(8)]
            
            for angle in scan_angles:
                # 模拟事件检测
                if random.random() > 0.9:  # 10%概率检测到事件
                    event_types = ['movement', 'intrusion', 'anomaly', 'debris']
                    
                    event = {
                        'type': random.choice(event_types),
                        'position': position + np.random.normal(0, 2, 3),
                        'angle': math.degrees(angle),
                        'confidence': random.uniform(0.7, 0.95),
                        'timestamp': time.time()
                    }
                    
                    result['events'].append(event)
                    result['events_detected'] = True
            
        except Exception as e:
            print(f"⚠️ 监视扫描失败: {e}")
        
        return result
    
    def capture_surveillance_image(self, position: np.ndarray, filename: str):
        """拍摄监控图像"""
        try:
            rgb_image = self.rov_controller.get_camera_image()
            if rgb_image is not None:
                image_data = {
                    'filename': filename,
                    'position': position.tolist(),
                    'timestamp': time.time(),
                    'size': rgb_image.shape if hasattr(rgb_image, 'shape') else 'unknown'
                }
                self.image_captures.append(image_data)
                print(f"📷 拍摄监控图像: {filename}")
            
        except Exception as e:
            print(f"⚠️ 图像拍摄失败: {e}")
    
    def capture_target_image(self, target: Dict, angle: float):
        """拍摄目标图像"""
        try:
            filename = f"target_{target['id']}_angle_{math.degrees(angle):.0f}"
            rgb_image = self.rov_controller.get_camera_image()
            
            if rgb_image is not None:
                image_data = {
                    'filename': filename,
                    'target_id': target['id'],
                    'angle': math.degrees(angle),
                    'timestamp': time.time()
                }
                self.image_captures.append(image_data)
                print(f"📷 拍摄目标图像: {filename}")
            
        except Exception as e:
            print(f"⚠️ 目标图像拍摄失败: {e}")
    
    def generate_inspection_report(self, target: Dict, inspection_result: Dict):
        """生成检查报告"""
        report = {
            'target': target,
            'inspection_result': inspection_result,
            'timestamp': time.time(),
            'report_id': f"inspection_{int(time.time())}"
        }
        
        print(f"📋 生成检查报告: {report['report_id']}")
        print(f"   目标: {target['name']}")
        print(f"   状态: {inspection_result['status']}")
        print(f"   质量评分: {inspection_result['quality_score']:.2f}")
        print(f"   发现问题: {len(inspection_result['issues_found'])}")
    
    def check_mission_interrupt(self) -> bool:
        """检查任务中断条件"""
        # 可以添加电量检查、通信状态检查等
        return False
    
    def save_mission_data(self, data: Dict, mission_type: str):
        """保存任务数据"""
        try:
            timestamp = int(time.time())
            filename = f"{mission_type}_{timestamp}.json"
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            
            print(f"💾 任务数据已保存: {filename}")
            
        except Exception as e:
            print(f"⚠️ 保存任务数据失败: {e}")
    
    def shutdown(self):
        """关闭任务控制器"""
        if self.rov_controller:
            self.rov_controller.shutdown()

class MissionDemoRunner:
    """任务演示运行器"""
    
    def __init__(self):
        self.mission_controller = AdvancedMissionController()
        self.available_missions = {
            'search_rescue': {
                'name': '搜索救援任务',
                'description': 'ROV搜索指定区域内的目标物体',
                'duration': 15,
                'func': self.demo_search_rescue
            },
            'underwater_inspection': {
                'name': '水下结构检查',
                'description': 'ROV检查水下结构的完整性',
                'duration': 12,
                'func': self.demo_underwater_inspection
            },
            'scientific_sampling': {
                'name': '科学采样任务', 
                'description': 'ROV在指定位置采集水样和沉积物',
                'duration': 10,
                'func': self.demo_scientific_sampling
            },
            'pipeline_following': {
                'name': '管道跟踪检查',
                'description': 'ROV跟踪管道路径并检查异常',
                'duration': 8,
                'func': self.demo_pipeline_following
            },
            'environmental_monitoring': {
                'name': '环境监控任务',
                'description': 'ROV监控水质和环境参数',
                'duration': 10,
                'func': self.demo_environmental_monitoring
            },
            'surveillance_patrol': {
                'name': '监视巡逻任务',
                'description': 'ROV按预定路线进行安全巡逻',
                'duration': 12,
                'func': self.demo_surveillance_patrol
            }
        }
    
    def demo_search_rescue(self):
        """搜索救援任务演示"""
        # 定义搜索区域
        area_min = np.array([-5, -5, -3])
        area_max = np.array([5, 5, -1])
        target_signatures = ['debris', 'object']
        
        return self.mission_controller.mission_search_rescue(
            (area_min, area_max), target_signatures
        )
    
    def demo_underwater_inspection(self):
        """水下检查任务演示"""
        inspection_targets = [
            {'name': '支柱A', 'position': [2, 0, -2], 'type': 'comprehensive', 'radius': 1.5},
            {'name': '横梁B', 'position': [0, 3, -2], 'type': 'visual', 'radius': 1.0},
            {'name': '接头C', 'position': [-2, 0, -2], 'type': 'detailed', 'radius': 0.8}
        ]
        
        return self.mission_controller.mission_underwater_inspection(inspection_targets)
    
    def demo_scientific_sampling(self):
        """科学采样任务演示"""
        sampling_locations = [
            {'name': '采样点1', 'position': [3, 2, -2], 'type': 'water', 'duration': 8},
            {'name': '采样点2', 'position': [-1, 4, -3], 'type': 'sediment', 'duration': 12},
            {'name': '采样点3', 'position': [1, -3, -2], 'type': 'water', 'duration': 6}
        ]
        
        return self.mission_controller.mission_scientific_sampling(sampling_locations)
    
    def demo_pipeline_following(self):
        """管道跟踪任务演示"""
        pipeline_start = np.array([-8, 0, -2])
        pipeline_direction = math.radians(45)  # 45度方向
        pipeline_length = 16.0  # 16米长
        
        return self.mission_controller.mission_pipeline_following(
            pipeline_start, pipeline_direction, pipeline_length
        )
    
    def demo_environmental_monitoring(self):
        """环境监控任务演示"""
        monitoring_stations = [
            {'name': '监控站A', 'position': [0, 0, -2], 'parameters': ['temperature', 'turbidity', 'ph']},
            {'name': '监控站B', 'position': [4, 0, -3], 'parameters': ['oxygen', 'salinity', 'pressure']},
            {'name': '监控站C', 'position': [0, 4, -2], 'parameters': ['temperature', 'oxygen', 'turbidity']}
        ]
        
        return self.mission_controller.mission_environmental_monitoring(
            monitoring_stations, monitoring_duration=45.0
        )
    
    def demo_surveillance_patrol(self):
        """监视巡逻任务演示"""
        patrol_route = [
            np.array([0, 0, -2]),
            np.array([5, 0, -2]),
            np.array([5, 5, -2]),
            np.array([0, 5, -2]),
            np.array([-5, 5, -2]),
            np.array([-5, 0, -2]),
            np.array([-5, -5, -2]),
            np.array([0, -5, -2])
        ]
        
        return self.mission_controller.mission_surveillance_patrol(patrol_route, patrol_cycles=2)
    
    def run_mission_demo(self, mission_name: str = 'all'):
        """运行任务演示"""
        print("🎖️ HAAV_Sim 高级任务演示程序")
        print("=" * 50)
        
        # 初始化ROV
        if not self.mission_controller.initialize_rov():
            return False
        
        try:
            if mission_name == 'all':
                # 运行所有任务演示
                success_count = 0
                total_missions = len(self.available_missions)
                
                for name, config in self.available_missions.items():
                    print(f"\n{'='*60}")
                    print(f"🎯 任务: {config['name']}")
                    print(f"📝 描述: {config['description']}")
                    print(f"⏱️ 预计时长: {config['duration']}分钟")
                    
                    success = config['func']()
                    if success:
                        success_count += 1
                        print(f"✅ 任务完成: {config['name']}")
                    else:
                        print(f"❌ 任务失败: {config['name']}")
                    
                    # 任务间暂停
                    print("\n⏸️ 任务间隔...")
                    time.sleep(5)
                
                print(f"\n🏆 所有任务演示完成!")
                print(f"📊 成功率: {success_count}/{total_missions} ({success_count/total_missions*100:.1f}%)")
                
            elif mission_name in self.available_missions:
                # 运行指定任务
                config = self.available_missions[mission_name]
                print(f"🎯 运行任务: {config['name']}")
                success = config['func']()
                return success
                
            else:
                print(f"❌ 未知任务: {mission_name}")
                print(f"💡 可用任务: {list(self.available_missions.keys())}")
                return False
            
            return True
            
        except KeyboardInterrupt:
            print("\n⏹️ 用户中断任务")
            return False
        
        finally:
            self.mission_controller.shutdown()

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='HAAV_Sim高级任务演示程序')
    parser.add_argument('--mission', '-m',
                       choices=list(MissionDemoRunner().available_missions.keys()) + ['all'],
                       default='all',
                       help='选择要运行的任务演示')
    
    args = parser.parse_args()
    
    # 创建并运行任务演示
    demo_runner = MissionDemoRunner()
    success = demo_runner.run_mission_demo(args.mission)
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())