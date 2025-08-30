#!/usr/bin/env python3
"""
交互式轨迹规划器
允许用户通过GUI界面设计自定义轨迹，实时预览并执行
"""

import sys
import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass, asdict
import threading

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'python'))

from rov_client import ROVController
from control_algorithms import ROVControlSystem, ControlMode
from trajectory_demos import TrajectoryGenerator, TrajectoryPoint

@dataclass
class WaypointGUI:
    """GUI航点定义"""
    id: int
    x: float
    y: float
    z: float
    yaw: float
    velocity: float
    hold_time: float
    trajectory_type: str = "linear"  # linear, curve, spiral, custom

class InteractiveTrajectoryPlanner:
    """交互式轨迹规划器"""
    
    def __init__(self):
        self.waypoints = []
        self.generated_trajectory = []
        self.rov_controller = None
        self.trajectory_generator = TrajectoryGenerator()
        
        # GUI组件
        self.root = None
        self.canvas_frame = None
        self.control_frame = None
        
        # 绘图相关
        self.fig = None
        self.ax = None
        self.waypoint_plots = []
        self.trajectory_plot = None
        
        # 状态变量
        self.current_waypoint_id = 0
        self.is_editing = False
        self.selected_waypoint = None
        
        # 配置
        self.workspace_bounds = {
            'x': [-10, 10],
            'y': [-10, 10], 
            'z': [-5, 0]
        }
        
    def create_gui(self):
        """创建GUI界面"""
        self.root = tk.Tk()
        self.root.title("HAAV_Sim 交互式轨迹规划器")
        self.root.geometry("1200x800")
        
        # 创建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 左侧控制面板
        self.create_control_panel(main_frame)
        
        # 右侧3D可视化
        self.create_visualization_panel(main_frame)
        
        # 底部状态栏
        self.create_status_bar(main_frame)
        
        # 绑定事件
        self.bind_events()
        
    def create_control_panel(self, parent):
        """创建控制面板"""
        control_frame = ttk.Frame(parent, width=300)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        control_frame.pack_propagate(False)
        
        # 标题
        title_label = ttk.Label(control_frame, text="轨迹规划控制器", 
                               font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # ROV连接状态
        self.create_connection_section(control_frame)
        
        # 航点管理
        self.create_waypoint_section(control_frame)
        
        # 轨迹生成
        self.create_trajectory_section(control_frame)
        
        # 执行控制
        self.create_execution_section(control_frame)
        
        # 文件操作
        self.create_file_section(control_frame)
        
    def create_connection_section(self, parent):
        """创建连接状态区域"""
        conn_frame = ttk.LabelFrame(parent, text="ROV连接", padding=10)
        conn_frame.pack(fill=tk.X, pady=5)
        
        self.connection_status = ttk.Label(conn_frame, text="未连接", 
                                         foreground="red")
        self.connection_status.pack()
        
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        self.connect_btn = ttk.Button(btn_frame, text="连接ROV", 
                                     command=self.connect_rov)
        self.connect_btn.pack(side=tk.LEFT)
        
        self.disconnect_btn = ttk.Button(btn_frame, text="断开连接", 
                                        command=self.disconnect_rov,
                                        state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.RIGHT)
        
    def create_waypoint_section(self, parent):
        """创建航点管理区域"""
        wp_frame = ttk.LabelFrame(parent, text="航点管理", padding=10)
        wp_frame.pack(fill=tk.X, pady=5)
        
        # 航点列表
        list_frame = ttk.Frame(wp_frame)
        list_frame.pack(fill=tk.X)
        
        self.waypoint_listbox = tk.Listbox(list_frame, height=6)
        self.waypoint_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.waypoint_listbox.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.waypoint_listbox.yview)
        
        # 航点参数输入
        param_frame = ttk.Frame(wp_frame)
        param_frame.pack(fill=tk.X, pady=5)
        
        # 位置输入
        pos_frame = ttk.Frame(param_frame)
        pos_frame.pack(fill=tk.X)
        
        ttk.Label(pos_frame, text="位置:").pack(side=tk.LEFT)
        
        self.x_var = tk.DoubleVar(value=0.0)
        self.y_var = tk.DoubleVar(value=0.0)
        self.z_var = tk.DoubleVar(value=-1.0)
        
        ttk.Label(pos_frame, text="X:").pack(side=tk.LEFT)
        ttk.Entry(pos_frame, textvariable=self.x_var, width=6).pack(side=tk.LEFT)
        
        ttk.Label(pos_frame, text="Y:").pack(side=tk.LEFT)
        ttk.Entry(pos_frame, textvariable=self.y_var, width=6).pack(side=tk.LEFT)
        
        ttk.Label(pos_frame, text="Z:").pack(side=tk.LEFT)
        ttk.Entry(pos_frame, textvariable=self.z_var, width=6).pack(side=tk.LEFT)
        
        # 其他参数
        other_frame = ttk.Frame(param_frame)
        other_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(other_frame, text="偏航:").pack(side=tk.LEFT)
        self.yaw_var = tk.DoubleVar(value=0.0)
        ttk.Entry(other_frame, textvariable=self.yaw_var, width=6).pack(side=tk.LEFT)
        
        ttk.Label(other_frame, text="速度:").pack(side=tk.LEFT)
        self.velocity_var = tk.DoubleVar(value=1.0)
        ttk.Entry(other_frame, textvariable=self.velocity_var, width=6).pack(side=tk.LEFT)
        
        # 轨迹类型选择
        type_frame = ttk.Frame(param_frame)
        type_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(type_frame, text="轨迹类型:").pack(side=tk.LEFT)
        self.trajectory_type_var = tk.StringVar(value="linear")
        type_combo = ttk.Combobox(type_frame, textvariable=self.trajectory_type_var,
                                 values=["linear", "curve", "spiral", "sine_wave"],
                                 state="readonly", width=10)
        type_combo.pack(side=tk.LEFT)
        
        # 航点操作按钮
        btn_frame = ttk.Frame(wp_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_frame, text="添加", command=self.add_waypoint).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="删除", command=self.delete_waypoint).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="修改", command=self.update_waypoint).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="清空", command=self.clear_waypoints).pack(side=tk.LEFT)
        
        # 快速添加按钮
        quick_frame = ttk.Frame(wp_frame)
        quick_frame.pack(fill=tk.X, pady=2)
        
        ttk.Button(quick_frame, text="方形轨迹", 
                  command=self.add_square_trajectory).pack(side=tk.LEFT)
        ttk.Button(quick_frame, text="圆形轨迹", 
                  command=self.add_circle_trajectory).pack(side=tk.LEFT)
        
    def create_trajectory_section(self, parent):
        """创建轨迹生成区域"""
        traj_frame = ttk.LabelFrame(parent, text="轨迹生成", padding=10)
        traj_frame.pack(fill=tk.X, pady=5)
        
        # 生成参数
        param_frame = ttk.Frame(traj_frame)
        param_frame.pack(fill=tk.X)
        
        ttk.Label(param_frame, text="时间步长:").pack(side=tk.LEFT)
        self.dt_var = tk.DoubleVar(value=0.1)
        ttk.Entry(param_frame, textvariable=self.dt_var, width=6).pack(side=tk.LEFT)
        
        ttk.Label(param_frame, text="平滑因子:").pack(side=tk.LEFT)
        self.smooth_var = tk.DoubleVar(value=0.1)
        ttk.Entry(param_frame, textvariable=self.smooth_var, width=6).pack(side=tk.LEFT)
        
        # 生成按钮
        btn_frame = ttk.Frame(traj_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_frame, text="生成轨迹", 
                  command=self.generate_trajectory).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="预览轨迹", 
                  command=self.preview_trajectory).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="清空轨迹", 
                  command=self.clear_trajectory).pack(side=tk.LEFT)
        
        # 轨迹信息
        self.trajectory_info = ttk.Label(traj_frame, text="轨迹: 未生成")
        self.trajectory_info.pack()
        
    def create_execution_section(self, parent):
        """创建执行控制区域"""
        exec_frame = ttk.LabelFrame(parent, text="执行控制", padding=10)
        exec_frame.pack(fill=tk.X, pady=5)
        
        # 执行参数
        param_frame = ttk.Frame(exec_frame)
        param_frame.pack(fill=tk.X)
        
        ttk.Label(param_frame, text="控制频率:").pack(side=tk.LEFT)
        self.control_freq_var = tk.DoubleVar(value=10.0)
        ttk.Entry(param_frame, textvariable=self.control_freq_var, width=6).pack(side=tk.LEFT)
        
        ttk.Label(param_frame, text="误差容限:").pack(side=tk.LEFT)
        self.error_tolerance_var = tk.DoubleVar(value=0.2)
        ttk.Entry(param_frame, textvariable=self.error_tolerance_var, width=6).pack(side=tk.LEFT)
        
        # 执行按钮
        btn_frame = ttk.Frame(exec_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        self.execute_btn = ttk.Button(btn_frame, text="执行轨迹", 
                                     command=self.execute_trajectory,
                                     state=tk.DISABLED)
        self.execute_btn.pack(side=tk.LEFT)
        
        self.pause_btn = ttk.Button(btn_frame, text="暂停", 
                                   command=self.pause_execution,
                                   state=tk.DISABLED)
        self.pause_btn.pack(side=tk.LEFT)
        
        self.stop_btn = ttk.Button(btn_frame, text="停止", 
                                  command=self.stop_execution,
                                  state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT)
        
        # 执行状态
        self.execution_status = ttk.Label(exec_frame, text="状态: 待机")
        self.execution_status.pack()
        
        # 进度条
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(exec_frame, variable=self.progress_var,
                                           maximum=100)
        self.progress_bar.pack(fill=tk.X, pady=2)
        
    def create_file_section(self, parent):
        """创建文件操作区域"""
        file_frame = ttk.LabelFrame(parent, text="文件操作", padding=10)
        file_frame.pack(fill=tk.X, pady=5)
        
        btn_frame = ttk.Frame(file_frame)
        btn_frame.pack(fill=tk.X)
        
        ttk.Button(btn_frame, text="保存轨迹", 
                  command=self.save_trajectory).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="加载轨迹", 
                  command=self.load_trajectory).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="导出数据", 
                  command=self.export_data).pack(side=tk.LEFT)
        
    def create_visualization_panel(self, parent):
        """创建3D可视化面板"""
        viz_frame = ttk.Frame(parent)
        viz_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 创建matplotlib图形
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 设置初始视图
        self.setup_3d_plot()
        
        # 嵌入到tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, viz_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 工具栏
        toolbar = NavigationToolbar2Tk(self.canvas, viz_frame)
        toolbar.update()
        
    def create_status_bar(self, parent):
        """创建状态栏"""
        status_frame = ttk.Frame(parent)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.status_label = ttk.Label(status_frame, text="就绪")
        self.status_label.pack(side=tk.LEFT)
        
        # ROV当前位置显示
        self.position_label = ttk.Label(status_frame, text="ROV位置: 未连接")
        self.position_label.pack(side=tk.RIGHT)
        
    def setup_3d_plot(self):
        """设置3D绘图"""
        self.ax.clear()
        
        # 设置坐标轴
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')  
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('ROV轨迹规划可视化')
        
        # 设置工作空间边界
        bounds = self.workspace_bounds
        self.ax.set_xlim(bounds['x'])
        self.ax.set_ylim(bounds['y'])
        self.ax.set_zlim(bounds['z'])
        
        # 绘制工作空间边界框
        self.draw_workspace_bounds()
        
        # 添加网格
        self.ax.grid(True)
        
        self.canvas.draw()
        
    def draw_workspace_bounds(self):
        """绘制工作空间边界"""
        bounds = self.workspace_bounds
        
        # 绘制边界线框
        vertices = [
            [bounds['x'][0], bounds['y'][0], bounds['z'][0]],  # 0
            [bounds['x'][1], bounds['y'][0], bounds['z'][0]],  # 1
            [bounds['x'][1], bounds['y'][1], bounds['z'][0]],  # 2
            [bounds['x'][0], bounds['y'][1], bounds['z'][0]],  # 3
            [bounds['x'][0], bounds['y'][0], bounds['z'][1]],  # 4
            [bounds['x'][1], bounds['y'][0], bounds['z'][1]],  # 5
            [bounds['x'][1], bounds['y'][1], bounds['z'][1]],  # 6
            [bounds['x'][0], bounds['y'][1], bounds['z'][1]]   # 7
        ]
        
        # 底面
        bottom_edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
        # 顶面  
        top_edges = [(4, 5), (5, 6), (6, 7), (7, 4)]
        # 垂直边
        vertical_edges = [(0, 4), (1, 5), (2, 6), (3, 7)]
        
        all_edges = bottom_edges + top_edges + vertical_edges
        
        for edge in all_edges:
            start, end = vertices[edge[0]], vertices[edge[1]]
            self.ax.plot3D([start[0], end[0]], [start[1], end[1]], 
                          [start[2], end[2]], 'k--', alpha=0.3)
        
    def bind_events(self):
        """绑定事件处理"""
        # 航点列表选择事件
        self.waypoint_listbox.bind('<<ListboxSelect>>', self.on_waypoint_select)
        
        # 关闭窗口事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 鼠标点击3D图形添加航点 (需要特殊处理)
        self.canvas.mpl_connect('button_press_event', self.on_canvas_click)
        
    def connect_rov(self):
        """连接ROV"""
        try:
            self.connection_status.config(text="连接中...", foreground="orange")
            self.root.update()
            
            self.rov_controller = ROVController()
            if self.rov_controller.initialize():
                self.connection_status.config(text="已连接", foreground="green")
                self.connect_btn.config(state=tk.DISABLED)
                self.disconnect_btn.config(state=tk.NORMAL)
                self.execute_btn.config(state=tk.NORMAL)
                
                # 启动位置更新线程
                self.start_position_update()
                
                messagebox.showinfo("成功", "ROV连接成功！")
            else:
                raise Exception("ROV初始化失败")
                
        except Exception as e:
            self.connection_status.config(text="连接失败", foreground="red")
            messagebox.showerror("错误", f"ROV连接失败: {e}")
    
    def disconnect_rov(self):
        """断开ROV连接"""
        try:
            if self.rov_controller:
                self.rov_controller.shutdown()
                self.rov_controller = None
                
            self.connection_status.config(text="未连接", foreground="red")
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            self.execute_btn.config(state=tk.DISABLED)
            self.position_label.config(text="ROV位置: 未连接")
            
        except Exception as e:
            messagebox.showerror("错误", f"断开连接失败: {e}")
    
    def start_position_update(self):
        """启动ROV位置更新线程"""
        def update_position():
            while self.rov_controller:
                try:
                    if self.rov_controller:
                        state = self.rov_controller.get_rov_state()
                        if state:
                            pos_text = f"ROV位置: ({state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f})"
                            self.position_label.config(text=pos_text)
                            
                            # 在3D图中显示ROV当前位置
                            self.update_rov_position_display(state.position)
                    
                    time.sleep(0.5)  # 2Hz更新频率
                    
                except Exception:
                    break
        
        threading.Thread(target=update_position, daemon=True).start()
    
    def update_rov_position_display(self, position):
        """更新3D图中的ROV位置显示"""
        # 清除之前的ROV位置标记
        for artist in self.ax.collections:
            if hasattr(artist, '_rov_marker'):
                artist.remove()
        
        # 添加新的ROV位置标记
        scatter = self.ax.scatter([position[0]], [position[1]], [position[2]], 
                                 c='red', s=100, marker='^', label='ROV当前位置')
        scatter._rov_marker = True
        
        self.canvas.draw_idle()
    
    def add_waypoint(self):
        """添加航点"""
        try:
            waypoint = WaypointGUI(
                id=self.current_waypoint_id,
                x=self.x_var.get(),
                y=self.y_var.get(),
                z=self.z_var.get(),
                yaw=math.radians(self.yaw_var.get()),  # 转换为弧度
                velocity=self.velocity_var.get(),
                hold_time=0.0,
                trajectory_type=self.trajectory_type_var.get()
            )
            
            self.waypoints.append(waypoint)
            self.current_waypoint_id += 1
            
            # 更新列表显示
            self.update_waypoint_listbox()
            
            # 更新3D显示
            self.update_3d_display()
            
            self.set_status(f"已添加航点 {waypoint.id}")
            
        except Exception as e:
            messagebox.showerror("错误", f"添加航点失败: {e}")
    
    def delete_waypoint(self):
        """删除选中的航点"""
        selection = self.waypoint_listbox.curselection()
        if not selection:
            messagebox.showwarning("警告", "请先选择要删除的航点")
            return
        
        index = selection[0]
        if 0 <= index < len(self.waypoints):
            removed_waypoint = self.waypoints.pop(index)
            self.update_waypoint_listbox()
            self.update_3d_display()
            self.set_status(f"已删除航点 {removed_waypoint.id}")
    
    def update_waypoint(self):
        """更新选中的航点"""
        selection = self.waypoint_listbox.curselection()
        if not selection:
            messagebox.showwarning("警告", "请先选择要修改的航点")
            return
        
        index = selection[0]
        if 0 <= index < len(self.waypoints):
            waypoint = self.waypoints[index]
            waypoint.x = self.x_var.get()
            waypoint.y = self.y_var.get()
            waypoint.z = self.z_var.get()
            waypoint.yaw = math.radians(self.yaw_var.get())
            waypoint.velocity = self.velocity_var.get()
            waypoint.trajectory_type = self.trajectory_type_var.get()
            
            self.update_waypoint_listbox()
            self.update_3d_display()
            self.set_status(f"已更新航点 {waypoint.id}")
    
    def clear_waypoints(self):
        """清空所有航点"""
        if self.waypoints and messagebox.askyesno("确认", "确定要清空所有航点吗?"):
            self.waypoints.clear()
            self.current_waypoint_id = 0
            self.update_waypoint_listbox()
            self.update_3d_display()
            self.set_status("已清空所有航点")
    
    def add_square_trajectory(self):
        """添加方形轨迹航点"""
        size = 2.0
        depth = -2.0
        
        square_points = [
            (0, 0, depth),
            (size, 0, depth),
            (size, size, depth),
            (0, size, depth),
            (0, 0, depth)
        ]
        
        for i, (x, y, z) in enumerate(square_points):
            waypoint = WaypointGUI(
                id=self.current_waypoint_id,
                x=x, y=y, z=z,
                yaw=i * math.pi / 2,  # 每个拐角转90度
                velocity=1.0,
                hold_time=1.0 if i < len(square_points)-1 else 0,  # 除了最后一点，其他点暂停1秒
                trajectory_type="linear"
            )
            self.waypoints.append(waypoint)
            self.current_waypoint_id += 1
        
        self.update_waypoint_listbox()
        self.update_3d_display()
        self.set_status("已添加方形轨迹")
    
    def add_circle_trajectory(self):
        """添加圆形轨迹航点"""
        radius = 2.0
        depth = -2.0
        num_points = 8
        
        for i in range(num_points + 1):  # +1 to close the circle
            angle = i * 2 * math.pi / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            waypoint = WaypointGUI(
                id=self.current_waypoint_id,
                x=x, y=y, z=depth,
                yaw=angle + math.pi/2,  # 切线方向
                velocity=1.0,
                hold_time=0.0,
                trajectory_type="curve"
            )
            self.waypoints.append(waypoint)
            self.current_waypoint_id += 1
        
        self.update_waypoint_listbox()
        self.update_3d_display()
        self.set_status("已添加圆形轨迹")
    
    def update_waypoint_listbox(self):
        """更新航点列表显示"""
        self.waypoint_listbox.delete(0, tk.END)
        
        for i, wp in enumerate(self.waypoints):
            text = f"WP{wp.id}: ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f}) | {wp.trajectory_type}"
            self.waypoint_listbox.insert(tk.END, text)
    
    def update_3d_display(self):
        """更新3D显示"""
        # 清除航点标记
        for plot in self.waypoint_plots:
            plot.remove()
        self.waypoint_plots.clear()
        
        # 清除轨迹线
        if self.trajectory_plot:
            for line in self.trajectory_plot:
                line.remove()
            self.trajectory_plot = None
        
        # 重新绘制
        self.setup_3d_plot()
        
        if not self.waypoints:
            return
        
        # 绘制航点
        x_coords = [wp.x for wp in self.waypoints]
        y_coords = [wp.y for wp in self.waypoints]
        z_coords = [wp.z for wp in self.waypoints]
        
        # 航点散点图
        scatter = self.ax.scatter(x_coords, y_coords, z_coords, 
                                 c='blue', s=50, label='航点')
        self.waypoint_plots.append(scatter)
        
        # 航点编号标注
        for wp in self.waypoints:
            text = self.ax.text(wp.x, wp.y, wp.z, f'  WP{wp.id}', fontsize=8)
            self.waypoint_plots.append(text)
        
        # 连接线
        if len(self.waypoints) > 1:
            line = self.ax.plot(x_coords, y_coords, z_coords, 
                               'b--', alpha=0.7, label='航点连线')[0]
            self.waypoint_plots.append(line)
        
        # 绘制生成的轨迹
        if self.generated_trajectory:
            traj_x = [p.x for p in self.generated_trajectory]
            traj_y = [p.y for p in self.generated_trajectory] 
            traj_z = [p.z for p in self.generated_trajectory]
            
            traj_line = self.ax.plot(traj_x, traj_y, traj_z,
                                    'r-', alpha=0.8, linewidth=2, label='生成轨迹')[0]
            self.waypoint_plots.append(traj_line)
        
        self.ax.legend()
        self.canvas.draw()
    
    def generate_trajectory(self):
        """生成轨迹"""
        if len(self.waypoints) < 2:
            messagebox.showwarning("警告", "至少需要2个航点才能生成轨迹")
            return
        
        try:
            self.generated_trajectory.clear()
            
            # 根据航点类型生成轨迹段
            for i in range(len(self.waypoints) - 1):
                start_wp = self.waypoints[i]
                end_wp = self.waypoints[i + 1]
                
                segment_traj = self.generate_trajectory_segment(start_wp, end_wp)
                
                # 避免重复添加连接点
                if i > 0 and segment_traj:
                    segment_traj = segment_traj[1:]
                
                self.generated_trajectory.extend(segment_traj)
            
            # 更新显示
            self.update_3d_display()
            
            # 更新信息显示
            total_time = len(self.generated_trajectory) * self.dt_var.get()
            self.trajectory_info.config(
                text=f"轨迹: {len(self.generated_trajectory)}点, {total_time:.1f}秒"
            )
            
            self.set_status("轨迹生成完成")
            
        except Exception as e:
            messagebox.showerror("错误", f"轨迹生成失败: {e}")
    
    def generate_trajectory_segment(self, start_wp: WaypointGUI, 
                                  end_wp: WaypointGUI) -> List[TrajectoryPoint]:
        """生成轨迹段"""
        start_pos = np.array([start_wp.x, start_wp.y, start_wp.z])
        end_pos = np.array([end_wp.x, end_wp.y, end_wp.z])
        
        distance = np.linalg.norm(end_pos - start_pos)
        duration = distance / start_wp.velocity
        
        if start_wp.trajectory_type == "linear":
            # 直线轨迹
            return self.trajectory_generator.generate_sine_wave(
                start_pos, end_pos, 0.0, 0.0, duration
            )
        
        elif start_wp.trajectory_type == "curve":
            # 曲线轨迹 (正弦波)
            amplitude = min(0.5, distance * 0.1)
            return self.trajectory_generator.generate_sine_wave(
                start_pos, end_pos, amplitude, 1.0, duration
            )
        
        elif start_wp.trajectory_type == "spiral":
            # 螺旋轨迹
            center = (start_pos + end_pos) / 2
            return self.trajectory_generator.generate_spiral_ascent(
                center, 0.5, 1.0, start_pos[2], end_pos[2], 1.0, duration
            )
        
        else:
            # 默认直线
            return self.trajectory_generator.generate_sine_wave(
                start_pos, end_pos, 0.0, 0.0, duration
            )
    
    def preview_trajectory(self):
        """预览轨迹"""
        if not self.generated_trajectory:
            self.generate_trajectory()
        
        if self.generated_trajectory:
            # 创建新窗口显示详细轨迹信息
            self.show_trajectory_details()
    
    def show_trajectory_details(self):
        """显示轨迹详细信息"""
        detail_window = tk.Toplevel(self.root)
        detail_window.title("轨迹详细信息")
        detail_window.geometry("400x600")
        
        # 创建文本框显示轨迹点信息
        text_frame = ttk.Frame(detail_window)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        text_widget = tk.Text(text_frame, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=text_widget.yview)
        text_widget.config(yscrollcommand=scrollbar.set)
        
        text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 填充轨迹信息
        text_widget.insert(tk.END, f"轨迹详细信息\n{'='*30}\n\n")
        text_widget.insert(tk.END, f"总点数: {len(self.generated_trajectory)}\n")
        text_widget.insert(tk.END, f"总时长: {len(self.generated_trajectory) * self.dt_var.get():.1f}秒\n\n")
        
        text_widget.insert(tk.END, "轨迹点详情:\n")
        for i, point in enumerate(self.generated_trajectory[::max(1, len(self.generated_trajectory)//50)]):
            text_widget.insert(tk.END, 
                f"#{i:4d}: ({point.x:6.2f}, {point.y:6.2f}, {point.z:6.2f}) "
                f"yaw={math.degrees(point.yaw):6.1f}° v={point.velocity:.1f}\n"
            )
        
        text_widget.config(state=tk.DISABLED)
    
    def clear_trajectory(self):
        """清空生成的轨迹"""
        self.generated_trajectory.clear()
        self.update_3d_display()
        self.trajectory_info.config(text="轨迹: 未生成")
        self.set_status("已清空轨迹")
    
    def execute_trajectory(self):
        """执行轨迹"""
        if not self.generated_trajectory:
            messagebox.showwarning("警告", "请先生成轨迹")
            return
        
        if not self.rov_controller:
            messagebox.showwarning("警告", "请先连接ROV")
            return
        
        # 在新线程中执行轨迹
        self.execution_thread = threading.Thread(
            target=self._execute_trajectory_worker,
            daemon=True
        )
        self.execution_thread.start()
        
        # 更新UI状态
        self.execute_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.NORMAL)
    
    def _execute_trajectory_worker(self):
        """轨迹执行工作线程"""
        try:
            self.execution_status.config(text="状态: 执行中")
            control_system = ROVControlSystem()
            control_system.set_control_mode(ControlMode.POSITION)
            
            dt = 1.0 / self.control_freq_var.get()
            error_tolerance = self.error_tolerance_var.get()
            
            for i, target_point in enumerate(self.generated_trajectory):
                # 检查是否需要停止或暂停
                if hasattr(self, 'stop_execution_flag') and self.stop_execution_flag:
                    break
                
                # 获取当前状态
                current_state = self.rov_controller.get_rov_state()
                if not current_state:
                    continue
                
                # 计算控制输出
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
                
                control_output = control_system.execute_control(current_dict, target_dict)
                pwm_values = self.rov_controller.wrench_to_pwm(control_output)
                self.rov_controller.set_thruster_pwm(pwm_values.tolist(), dt)
                
                # 更新进度
                progress = (i + 1) / len(self.generated_trajectory) * 100
                self.progress_var.set(progress)
                
                # 计算误差
                error = np.linalg.norm(np.array([target_point.x, target_point.y, target_point.z]) - 
                                     current_state.position)
                
                self.set_status(f"执行中: 点{i+1}/{len(self.generated_trajectory)}, 误差{error:.3f}m")
                
                time.sleep(dt)
            
            # 执行完成
            self.rov_controller.emergency_stop()
            self.execution_status.config(text="状态: 完成")
            self.set_status("轨迹执行完成")
            
        except Exception as e:
            self.execution_status.config(text="状态: 错误")
            self.set_status(f"执行错误: {e}")
            
        finally:
            # 恢复UI状态
            self.execute_btn.config(state=tk.NORMAL)
            self.pause_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.DISABLED)
            self.progress_var.set(0)
    
    def pause_execution(self):
        """暂停执行"""
        # 实现暂停逻辑
        self.set_status("执行已暂停")
    
    def stop_execution(self):
        """停止执行"""
        self.stop_execution_flag = True
        if self.rov_controller:
            self.rov_controller.emergency_stop()
        self.set_status("执行已停止")
    
    def save_trajectory(self):
        """保存轨迹到文件"""
        if not self.waypoints:
            messagebox.showwarning("警告", "没有航点可保存")
            return
        
        filename = filedialog.asksaveasfilename(
            title="保存轨迹",
            filetypes=[("JSON文件", "*.json"), ("所有文件", "*.*")],
            defaultextension=".json"
        )
        
        if filename:
            try:
                data = {
                    'waypoints': [asdict(wp) for wp in self.waypoints],
                    'generated_trajectory': [asdict(tp) for tp in self.generated_trajectory] if self.generated_trajectory else [],
                    'parameters': {
                        'dt': self.dt_var.get(),
                        'smooth_factor': self.smooth_var.get(),
                        'control_frequency': self.control_freq_var.get(),
                        'error_tolerance': self.error_tolerance_var.get()
                    }
                }
                
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                
                messagebox.showinfo("成功", f"轨迹已保存到: {filename}")
                self.set_status(f"轨迹已保存: {filename}")
                
            except Exception as e:
                messagebox.showerror("错误", f"保存失败: {e}")
    
    def load_trajectory(self):
        """从文件加载轨迹"""
        filename = filedialog.askopenfilename(
            title="加载轨迹",
            filetypes=[("JSON文件", "*.json"), ("所有文件", "*.*")]
        )
        
        if filename:
            try:
                with open(filename, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                
                # 清空当前数据
                self.waypoints.clear()
                self.generated_trajectory.clear()
                
                # 加载航点
                for wp_data in data.get('waypoints', []):
                    wp = WaypointGUI(**wp_data)
                    self.waypoints.append(wp)
                
                # 加载生成的轨迹
                for tp_data in data.get('generated_trajectory', []):
                    tp = TrajectoryPoint(**tp_data)
                    self.generated_trajectory.append(tp)
                
                # 加载参数
                params = data.get('parameters', {})
                self.dt_var.set(params.get('dt', 0.1))
                self.smooth_var.set(params.get('smooth_factor', 0.1))
                self.control_freq_var.set(params.get('control_frequency', 10.0))
                self.error_tolerance_var.set(params.get('error_tolerance', 0.2))
                
                # 更新显示
                self.update_waypoint_listbox()
                self.update_3d_display()
                
                if self.waypoints:
                    self.current_waypoint_id = max(wp.id for wp in self.waypoints) + 1
                
                # 更新轨迹信息
                if self.generated_trajectory:
                    total_time = len(self.generated_trajectory) * self.dt_var.get()
                    self.trajectory_info.config(
                        text=f"轨迹: {len(self.generated_trajectory)}点, {total_time:.1f}秒"
                    )
                
                messagebox.showinfo("成功", f"轨迹已从 {filename} 加载")
                self.set_status(f"轨迹已加载: {filename}")
                
            except Exception as e:
                messagebox.showerror("错误", f"加载失败: {e}")
    
    def export_data(self):
        """导出数据"""
        if not self.generated_trajectory:
            messagebox.showwarning("警告", "没有轨迹数据可导出")
            return
        
        filename = filedialog.asksaveasfilename(
            title="导出数据",
            filetypes=[("CSV文件", "*.csv"), ("JSON文件", "*.json")],
            defaultextension=".csv"
        )
        
        if filename:
            try:
                if filename.endswith('.csv'):
                    # 导出为CSV格式
                    import csv
                    with open(filename, 'w', newline='', encoding='utf-8') as f:
                        writer = csv.writer(f)
                        writer.writerow(['Index', 'X', 'Y', 'Z', 'Yaw', 'Velocity', 'Timestamp'])
                        
                        for i, tp in enumerate(self.generated_trajectory):
                            writer.writerow([i, tp.x, tp.y, tp.z, 
                                           math.degrees(tp.yaw), tp.velocity, tp.timestamp])
                else:
                    # 导出为JSON格式  
                    data = {
                        'trajectory_points': [asdict(tp) for tp in self.generated_trajectory],
                        'summary': {
                            'total_points': len(self.generated_trajectory),
                            'duration': len(self.generated_trajectory) * self.dt_var.get(),
                            'export_time': time.time()
                        }
                    }
                    
                    with open(filename, 'w', encoding='utf-8') as f:
                        json.dump(data, f, indent=2, ensure_ascii=False)
                
                messagebox.showinfo("成功", f"数据已导出到: {filename}")
                self.set_status(f"数据已导出: {filename}")
                
            except Exception as e:
                messagebox.showerror("错误", f"导出失败: {e}")
    
    def on_waypoint_select(self, event):
        """航点选择事件处理"""
        selection = self.waypoint_listbox.curselection()
        if selection:
            index = selection[0]
            if 0 <= index < len(self.waypoints):
                wp = self.waypoints[index]
                
                # 更新输入框
                self.x_var.set(wp.x)
                self.y_var.set(wp.y) 
                self.z_var.set(wp.z)
                self.yaw_var.set(math.degrees(wp.yaw))
                self.velocity_var.set(wp.velocity)
                self.trajectory_type_var.set(wp.trajectory_type)
                
                self.selected_waypoint = wp
    
    def on_canvas_click(self, event):
        """画布点击事件处理"""
        # TODO: 实现3D点击添加航点功能
        # 这需要复杂的3D投影计算
        pass
    
    def set_status(self, message: str):
        """设置状态栏消息"""
        self.status_label.config(text=message)
    
    def on_closing(self):
        """窗口关闭事件处理"""
        if self.rov_controller:
            self.disconnect_rov()
        
        self.root.destroy()
    
    def run(self):
        """运行GUI应用"""
        self.create_gui()
        
        print("🎯 启动交互式轨迹规划器...")
        print("💡 使用说明:")
        print("   1. 首先连接ROV")
        print("   2. 添加航点定义轨迹")  
        print("   3. 生成平滑轨迹")
        print("   4. 预览并执行轨迹")
        print("   5. 保存/加载轨迹文件")
        
        self.root.mainloop()

def main():
    """主函数"""
    print("🚀 HAAV_Sim 交互式轨迹规划器")
    print("=" * 40)
    
    try:
        planner = InteractiveTrajectoryPlanner()
        planner.run()
        
    except Exception as e:
        print(f"❌ 程序启动失败: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())