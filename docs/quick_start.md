# HAAV_Sim Windows 最小实现 - 快速开始指南

> **目标**: 30分钟内完成水下ROV仿真环境的完整部署和测试
> 
> **适用系统**: Windows 10/11 (64位)

## 30分钟快速部署

### 第一阶段: 环境准备 (10分钟)

#### 1. 下载项目
```batch
git clone https://github.com/your-repo/HAAV_Sim.git
cd HAAV_Sim
```

#### 2. 自动安装依赖
```batch
# 以管理员身份运行
setup\install_dependencies.bat
```

**自动安装内容**:
- Chocolatey 包管理器
- Git 版本控制工具  
- Python 3.8+ 运行环境
- Visual Studio Build Tools
- .NET 6.0 Runtime
- CMake 构建工具

#### 3. 手动安装UE5
1. 打开 Epic Games Launcher (脚本会自动打开下载页面)
2. 登录/注册 Epic Games 账号
3. 在"虚幻引擎"选项卡中安装 **UE 5.1.1**

### 第二阶段: 项目构建 (15分钟)

#### 4. 构建项目
```batch
setup\build_project.bat
```

**自动构建内容**:
- 下载并编译 AirSim 库
- 创建 UE5 项目文件
- 安装 AirSim 插件
- 生成 Visual Studio 项目
- 创建默认配置文件

#### 5. 配置Python环境
```batch
setup\setup_environment.bat
```

**自动配置内容**:
- 创建Python虚拟环境
- 安装所有依赖包
- 安装AirSim Python API
- 运行环境测试

### 第三阶段: 验证测试 (5分钟)

#### 6. 启动仿真
```batch
launch_simulation.bat
```

#### 7. 运行功能测试
```batch
# 激活Python环境
call activate_env.bat

# 运行完整演示
python examples\complete_rov_demo.py --scenario basic_movement
```

---

## 核心功能验证

### 基础功能测试

#### 1. 连接测试
```python
from src.python.rov_client import ROVController

# 创建ROV控制器
rov = ROVController()
if rov.initialize():
    print("ROV系统连接成功")
    
    # 获取当前状态
    state = rov.get_rov_state()
    print(f"位置: {state.position}")
    print(f"深度: {state.depth}m")
```

#### 2. 推进器控制测试
```python
# 测试推进器PWM控制
pwm_values = [0.5, 0.5, 0.5, 0.5, 0.6, 0.4, 0.6, 0.4]  # 前进
success = rov.set_thruster_pwm(pwm_values, 3.0)
print(f"推进器控制: {'成功' if success else '失败'}")
```

#### 3. 传感器数据测试
```python
# 获取相机图像
rgb_image = rov.get_camera_image("front_center")
print(f"RGB图像: {'获取成功' if rgb_image is not None else '获取失败'}")

# 获取IMU数据
imu_data = rov.get_imu_data()
print(f"IMU数据: {'获取成功' if imu_data else '获取失败'}")
```

### 高级功能测试

#### 1. 轨迹跟踪测试
```batch
python examples\complete_rov_demo.py --scenario trajectory_following
```

#### 2. 定点保持测试
```batch
python examples\complete_rov_demo.py --scenario station_keeping
```

#### 3. 障碍物避让测试
```batch
python examples\complete_rov_demo.py --scenario obstacle_avoidance
```

#### 4. 完整任务测试
```batch
python examples\complete_rov_demo.py --scenario complete_mission
```

---

## 功能清单验证

### 必备功能检查表

- [ ] **ROV物理仿真**: BlueROV2 Heavy 8推进器模型
- [ ] **水动力学模拟**: 阻力、浮力、流体动力学
- [ ] **推进器PWM控制**: 直接电机控制接口
- [ ] **传感器数据获取**: 相机、IMU、深度传感器
- [ ] **位置控制算法**: PID控制器
- [ ] **轨迹跟踪算法**: 路径规划和跟踪
- [ ] **定点保持功能**: 精确位置控制
- [ ] **障碍物避让**: 人工势场法
- [ ] **水下视觉渲染**: UE5 Water System
- [ ] **完整Python API**: 全功能控制接口

### 控制功能检查

```python
# 运行完整功能检查
python examples\complete_rov_demo.py --scenario all
```

**预期结果**:
- 基础移动演示: ROV响应推进器命令
- 轨迹跟踪演示: ROV跟踪预设路径  
- 传感器采集演示: 成功获取各类传感器数据
- 定点保持演示: ROV稳定保持目标位置
- 障碍物避让演示: ROV自主规避障碍物
- 完整任务演示: ROV执行复合任务

---

## 故障排除

### 常见问题

#### 1. UE5启动失败
**症状**: 双击.uproject文件无反应或报错
**解决**:
```batch
# 检查UE5安装
setup\verify_environment.bat

# 重新生成项目文件
cd unreal
"C:\Program Files\Epic Games\UE_5.1\Engine\Binaries\DotNET\UnrealBuildTool.exe" -projectfiles -project="Underwater.uproject" -game -rocket
```

#### 2. Python连接失败
**症状**: `airsim.VehicleClient()` 连接超时
**解决**:
```python
# 确认UE5项目正在运行
# 检查AirSim配置
import json
with open(r"C:\Users\你的用户名\Documents\AirSim\settings.json") as f:
    config = json.load(f)
    print("SimMode:", config.get("SimMode"))  # 应该是 "Rov"
```

#### 3. 推进器不响应
**症状**: `moveByMotorPWMsAsync` 调用成功但ROV不动
**解决**:
```python
# 检查ARM状态
client.armDisarm(True)
client.enableApiControl(True)

# 检查PWM值范围
pwm_values = [0.5, 0.5, 0.5, 0.5, 0.55, 0.45, 0.55, 0.45]  # 0.5为中性
```

#### 4. 性能问题
**症状**: UE5运行卡顿，帧率低
**解决**:
- 降低渲染质量设置
- 关闭不必要的特效
- 确保显卡驱动最新
- 增加虚拟内存

### 诊断工具

#### 系统诊断
```batch
setup\diagnose_system.bat
```

#### 环境测试
```batch
python test_environment.py
```

#### 详细日志
```batch
# 查看构建日志
type setup\build_log.txt

# 查看环境日志  
type setup\environment_log.txt
```

---

## 进阶使用

### 自定义开发

#### 1. 修改ROV参数
编辑 `config/settings.json`:
```json
{
  "Vehicles": {
    "BlueROV": {
      "WaterDynamics": {
        "LinearDrag": [2.0, 2.0, 2.5],
        "AngularDrag": [1.5, 1.5, 1.0],
        "BuoyancyForce": 9.8
      }
    }
  }
}
```

#### 2. 添加新传感器
```json
{
  "Sensors": {
    "lidar": {
      "SensorType": 6,
      "Enabled": true,
      "NumberOfChannels": 16,
      "Range": 10
    }
  }
}
```

#### 3. 扩展控制算法
```python
from src.python.control_algorithms import ROVControlSystem

class MyCustomController(ROVControlSystem):
    def my_custom_algorithm(self):
        # 实现你的控制算法
        pass
```

### 性能优化

#### 1. 图形优化
- 编辑器 → 项目设置 → 渲染
- 降低阴影质量、关闭光线追踪
- 使用"可扩展性"预设调整性能

#### 2. 仿真优化
```python
# 降低控制频率
dt = 0.05  # 20Hz instead of 100Hz

# 批量处理命令
commands = []
for i in range(10):
    commands.append(calculate_pwm())
# 批量发送
```

#### 3. 内存优化
- 限制同时采集的传感器数据
- 定期清理图像缓存
- 使用适当的数据类型

---

## 性能基准

### 最小系统要求验证

| 组件 | 最小要求 | 推荐配置 | 测试方法 |
|------|----------|----------|----------|
| CPU | Intel i5-8400 | Intel i7-10700K | `wmic cpu get name` |
| GPU | GTX 1060 6GB | RTX 3060 12GB | `wmic path win32_VideoController get name` |
| RAM | 16GB | 32GB | `wmic computersystem get TotalPhysicalMemory` |
| 存储 | 20GB 可用 | 50GB SSD | `dir C:` |

### 性能测试

#### 1. 仿真性能测试
```python
import time
start_time = time.time()
# 运行1000步仿真
for i in range(1000):
    state = rov.get_rov_state()
    # 控制计算
    
elapsed = time.time() - start_time
fps = 1000 / elapsed
print(f"仿真性能: {fps:.1f} FPS")
```

#### 2. 控制延迟测试
```python
import time
latencies = []
for i in range(100):
    start = time.time()
    rov.set_thruster_pwm([0.5]*8, 0.1)
    latencies.append(time.time() - start)

avg_latency = sum(latencies) / len(latencies)
print(f"平均控制延迟: {avg_latency*1000:.2f}ms")
```

### 预期性能指标

- **仿真帧率**: 30+ FPS (推荐配置下60+ FPS)
- **控制延迟**: <50ms (局域网环境)
- **内存使用**: <8GB (UE5编辑器 + Python)
- **启动时间**: <5分钟 (从冷启动到可用)

---

## 成功标准

当你完成快速开始指南后，应该能够:

1. **UE5仿真正常运行**: 看到水下环境和ROV模型
2. **Python API连接成功**: 无连接错误
3. **推进器控制响应**: ROV能够移动
4. **传感器数据正常**: 能获取图像和IMU数据
5. **轨迹跟踪工作**: ROV能跟踪预设路径
6. **完整演示运行**: 所有6个演示场景正常执行

**如果以上所有项目都通过，恭喜你！HAAV_Sim Windows最小实现部署成功！**

---

**下一步**: 查看 [API参考文档](api_reference.md) 了解详细的编程接口，或运行更高级的演示程序探索完整功能。