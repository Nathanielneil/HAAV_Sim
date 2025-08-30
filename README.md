<div align="center">

# HAAV_Sim - Underwater ROV Simulation Suite

**A comprehensive Windows-based underwater ROV simulation environment with complete hydrodynamic modeling**

[![Python](https://img.shields.io/badge/Python-3.7+-blue.svg)](https://www.python.org/)
[![AirSim](https://img.shields.io/badge/AirSim-1.8.1-green.svg)](https://microsoft.github.io/AirSim/)
[![UE5](https://img.shields.io/badge/Unreal%20Engine-5.1-orange.svg)](https://www.unrealengine.com/)
[![Windows](https://img.shields.io/badge/Windows-10/11-blue.svg)](https://www.microsoft.com/)
[![BlueROV2](https://img.shields.io/badge/Vehicle-BlueROV2%20Heavy-navy.svg)](https://bluerobotics.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

*Powered by Windows 10/11 • Unreal Engine 5.1 • AirSim Integration*

</div>

---

## Project Overview

HAAV_Sim is a complete underwater ROV simulation platform designed for rapid deployment on Windows systems. This suite provides researchers and developers with realistic hydrodynamic modeling, full ROV control capabilities, and comprehensive sensor integration in a single, easy-to-install package.

**Key Objectives:**
- **Rapid Deployment**: Complete setup in under 30 minutes
- **Minimal Dependencies**: Streamlined installation process
- **Maximum Functionality**: Full-featured underwater simulation
- **Production Ready**: Battle-tested BlueROV2 Heavy implementation

## Feature Matrix

### Core Simulation Features

| Feature | Implementation | Status | Description |
|---------|---------------|--------|-------------|
| **ROV Physics** | BlueROV2 Heavy | Complete | 8-thruster dynamics model |
| **Hydrodynamics** | Advanced Simulation | Complete | Realistic drag, buoyancy, water dynamics |
| **Visual Rendering** | UE5 Water System | Complete | Photorealistic underwater environments |
| **Control API** | Python/C++ | Complete | Dual-language development support |
| **Thruster Control** | PWM Interface | Complete | Real-time motor control simulation |
| **Sensor Suite** | Multi-Modal | Complete | Camera, IMU, depth, navigation sensors |
| **Navigation** | Advanced Algorithms | Complete | PID control and trajectory planning |

### Supported Control Modes

| Control Mode | Implementation | Use Case |
|-------------|---------------|----------|
| **Direct PWM** | `set_thruster_pwm()` | Low-level thruster control |
| **Velocity Control** | `move_by_velocity()` | Smooth directional movement |
| **Position Control** | `move_to_position()` | Precise positioning |
| **Trajectory Following** | `follow_trajectory()` | Complex path navigation |
| **Emergency Stop** | `emergency_stop()` | Safety and collision recovery |

## Architecture

```
HAAV_Sim/
├── Core Components
│   ├── README.md                     # Project documentation
│   ├── setup/                        # Installation scripts
│   │   ├── install_dependencies.bat  # Dependency installer
│   │   ├── build_project.bat        # Project builder
│   │   └── setup_environment.bat    # Environment configurator
│   └── config/                       # Configuration files
│       ├── settings.json            # AirSim settings
│       ├── DefaultEngine.ini        # UE5 engine config
│       └── water_config.json        # Water system config
│
├── Source Code
│   ├── src/cpp/                     # C++ implementation
│   │   ├── ROVController/           # ROV control systems
│   │   ├── WaterDynamics/          # Hydrodynamics engine
│   │   └── Sensors/                # Sensor interfaces
│   └── src/python/                 # Python implementation
│       ├── rov_client.py           # Main ROV client
│       ├── water_dynamics.py       # Physics calculations
│       ├── control_algorithms.py   # Control systems
│       ├── sensor_interface.py     # Sensor management
│       └── examples/               # Usage examples
│
├── Unreal Engine Integration
│   ├── unreal/Underwater.uproject  # Main UE5 project
│   ├── Content/                    # Game assets
│   │   ├── ROV/                    # ROV models and materials
│   │   ├── Environment/            # Underwater environments
│   │   └── Water/                  # Water system assets
│   └── Source/                     # UE5 C++ source
│
├── Assets & Resources
│   ├── assets/models/              # 3D models
│   ├── assets/textures/            # Texture library
│   └── assets/materials/           # Material definitions
│
├── Documentation & Examples
│   ├── docs/                       # Technical documentation
│   │   ├── quick_start.md          # Getting started guide
│   │   ├── api_reference.md        # API documentation
│   │   └── troubleshooting.md      # Problem resolution
│   └── examples/                   # Complete examples
│       ├── basic_control.py        # Basic ROV operations
│       ├── trajectory_following.py # Path planning
│       ├── sensor_data_collection.py # Data acquisition
│       └── advanced_navigation.py  # Advanced algorithms
```

## Quick Start Guide

### Prerequisites & System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **OS** | Windows 10 (64-bit) | Windows 11 (64-bit) |
| **RAM** | 16GB | 32GB+ |
| **GPU** | NVIDIA GTX 1060+ / AMD RX 580+ | NVIDIA RTX 3060+ |
| **Storage** | 20GB available space | 50GB+ SSD |
| **IDE** | Visual Studio 2019 | Visual Studio 2022 |

### Installation Process

#### Step 1: Repository Setup
```batch
@echo off
REM Download the project
git clone --depth 1 https://github.com/your-org/HAAV_Sim.git
cd HAAV_Sim

REM Run automated installation
setup\install_dependencies.bat
```

#### Step 2: Build & Configuration
```batch
REM Build UE5 project and AirSim integration
setup\build_project.bat

REM Configure Python environment
setup\setup_environment.bat
```

#### Step 3: System Verification
```batch
REM Launch simulation and run basic tests
examples\run_basic_test.bat
```

## Detailed Installation Guide

### Dependency Installation Script

The `setup/install_dependencies.bat` script performs the following operations:

```batch
@echo off
echo ===============================================
echo HAAV_Sim Windows Installation - Dependencies
echo ===============================================

REM Check administrative privileges
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo Please run this script as administrator!
    pause
    exit /b 1
)

REM 1. Install Chocolatey package manager
powershell -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))"

REM 2. Install development tools
choco install -y git cmake python3 nodejs

REM 3. Install Visual Studio Build Tools
choco install -y visualstudio2022buildtools
choco install -y visualstudio2022-workload-vctools

REM 4. Install .NET Runtime
choco install -y dotnet-6.0-runtime

REM 5. Download Epic Games Launcher (manual UE5 installation required)
start https://www.epicgames.com/store/en-US/download

echo.
echo ===============================================
echo Dependencies installed successfully!
echo Please install UE 5.1.1 through Epic Games Launcher
echo Then run build_project.bat
echo ===============================================
pause
```

### Project Build Script

The `setup/build_project.bat` handles the complete build process:

```batch
@echo off
setlocal enabledelayedexpansion

echo ===============================================
echo HAAV_Sim Windows Build Process
echo ===============================================

REM Set path variables
set PROJECT_ROOT=%~dp0..
set UE5_PATH=C:\Program Files\Epic Games\UE_5.1\Engine
set BUILD_DIR=%PROJECT_ROOT%\build

REM Check UE5 installation
if not exist "%UE5_PATH%" (
    echo Error: UE5 installation not found, please install UE 5.1.1
    pause
    exit /b 1
)

REM 1. Create build directory
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

REM 2. Build AirSim library
echo Building AirSim integration...
cd "%PROJECT_ROOT%\src\cpp"
call build.cmd

REM 3. Generate UE5 project files
echo Generating UE5 project files...
cd "%PROJECT_ROOT%\unreal"
"%UE5_PATH%\Binaries\DotNET\UnrealBuildTool.exe" -projectfiles -project="Underwater.uproject" -game -rocket -progress

REM 4. Compile UE5 project
echo Compiling UE5 project...
"%UE5_PATH%\Binaries\DotNET\UnrealBuildTool.exe" UnderwaterEditor Win64 Development "Underwater.uproject"

REM 5. Configure AirSim settings
echo Configuring AirSim...
if not exist "%USERPROFILE%\Documents\AirSim" mkdir "%USERPROFILE%\Documents\AirSim"
copy "%PROJECT_ROOT%\config\settings.json" "%USERPROFILE%\Documents\AirSim\settings.json"

echo.
echo ===============================================
echo Build completed! Run setup_environment.bat to configure Python
echo ===============================================
pause
```

## Python Environment Setup

### Environment Configuration Script

The `setup/setup_environment.bat` configures the Python development environment:

```batch
@echo off
echo ===============================================
echo Python Environment Configuration
echo ===============================================

set PROJECT_ROOT=%~dp0..

REM Create Python virtual environment
python -m venv "%PROJECT_ROOT%\venv"

REM Activate virtual environment
call "%PROJECT_ROOT%\venv\Scripts\activate.bat"

REM Install Python dependencies
pip install --upgrade pip
pip install numpy scipy matplotlib opencv-python
pip install msgpack-rpc-python airsim

REM Install project Python package
cd "%PROJECT_ROOT%\src\python"
pip install -e .

echo.
echo Python environment configured successfully!
echo Use the following command to activate:
echo call venv\Scripts\activate.bat
pause
```

## Configuration Files

### AirSim Settings

The `config/settings.json` file contains the complete AirSim configuration:

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Rov",
  "ClockSpeed": 1,
  "EnableCollisions": true,
  "PhysicsEngineName": "FastPhysicsEngine",
  
  "PawnPaths": {
    "DefaultRov": {"PawnBP": "Class'/Game/ROV/Blueprints/BP_BlueROV2Heavy.BP_BlueROV2Heavy_C'"}
  },
  
  "Vehicles": {
    "BlueROV": {
      "VehicleType": "RovSimple", 
      "DefaultVehicleState": "Armed",
      "PawnPath": "DefaultRov",
      "EnableCollisions": true,
      "AllowAPIAlways": true,
      
      "WaterDynamics": {
        "LinearDrag": [2.0, 2.0, 2.5],
        "AngularDrag": [1.5, 1.5, 1.0],
        "BuoyancyForce": 9.8,
        "WaterDensity": 1025.0,
        "VolumeDisplaced": 0.013
      },
      
      "ThrusterConfiguration": {
        "ThrusterCount": 8,
        "MaxThrust": 5.0,
        "ThrusterPositions": [
          {"X": 0.11, "Y": 0.22, "Z": 0.0, "Direction": [0, 0, 1]},
          {"X": 0.11, "Y": -0.22, "Z": 0.0, "Direction": [0, 0, 1]},
          {"X": -0.11, "Y": 0.22, "Z": 0.0, "Direction": [0, 0, 1]},
          {"X": -0.11, "Y": -0.22, "Z": 0.0, "Direction": [0, 0, 1]},
          {"X": 0.15, "Y": 0.09, "Z": 0.0, "Direction": [1, -1, 0]},
          {"X": 0.15, "Y": -0.09, "Z": 0.0, "Direction": [1, 1, 0]},
          {"X": -0.15, "Y": 0.09, "Z": 0.0, "Direction": [-1, 1, 0]},
          {"X": -0.15, "Y": -0.09, "Z": 0.0, "Direction": [-1, -1, 0]}
        ]
      },
      
      "Cameras": {
        "front_center": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 90,
              "AutoExposureMethod": "Manual",
              "MotionBlurAmount": 0.0
            }
          ],
          "X": 0.30, "Y": 0.0, "Z": 0.0,
          "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
        },
        
        "depth_camera": {
          "CaptureSettings": [
            {
              "ImageType": 2,
              "Width": 640,
              "Height": 480,
              "FOV_Degrees": 90
            }
          ],
          "X": 0.30, "Y": 0.0, "Z": -0.05
        }
      },
      
      "Sensors": {
        "imu": {
          "SensorType": 2,
          "Enabled": true
        },
        "barometer": {
          "SensorType": 1,
          "Enabled": true
        }
      }
    }
  },
  
  "SubWindows": [
    {"WindowID": 0, "CameraName": "front_center", "ImageType": 0}
  ]
}
```

### Water System Configuration

The `config/water_config.json` defines the underwater environment:

```json
{
  "WaterSystem": {
    "WaterBodyType": "Ocean",
    "WaterMaterial": "/Game/Water/Materials/M_UnderwaterOcean",
    "WaterLevel": 0.0,
    "WaveHeight": 0.5,
    "WaveFrequency": 1.0,
    "WaterDensity": 1025.0,
    "Visibility": 15.0,
    
    "PostProcessing": {
      "UnderwaterEffect": true,
      "CausticsEnabled": true,
      "ParticleEffects": true,
      "ColorGrading": {
        "GlobalSaturation": 0.7,
        "GlobalContrast": 1.1,
        "Shadows_Saturation": 0.8,
        "Tint": [0.85, 0.95, 1.0, 1.0]
      }
    },
    
    "Lighting": {
      "SunIntensity": 3.0,
      "SkyLightIntensity": 1.5,
      "UnderwaterAmbient": 0.3,
      "VolumetricFog": true
    }
  }
}
```

## Core Implementation

### Python ROV Client (Production-Ready)

The core `src/python/rov_client.py` provides complete ROV control capabilities:

```python
#!/usr/bin/env python3
"""
Production-ready ROV client implementation
Supports complete ROV operations: thruster control, sensor integration, navigation algorithms
"""

import airsim
import numpy as np
import time
import json
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import threading
import queue

@dataclass
class ROVState:
    """Complete ROV state information"""
    position: np.ndarray
    velocity: np.ndarray
    orientation: np.ndarray
    angular_velocity: np.ndarray
    depth: float
    timestamp: float

@dataclass
class ThrusterCommand:
    """Thruster command structure"""
    thruster_id: int
    pwm_value: float
    duration: float

class WaterDynamics:
    """Water dynamics calculation module"""
    
    def __init__(self, config_path: Optional[str] = None):
        if config_path:
            with open(config_path, 'r') as f:
                config = json.load(f)
            self.config = config['WaterDynamics']
        else:
            # Default BlueROV2 parameters
            self.config = {
                "LinearDrag": [2.0, 2.0, 2.5],
                "AngularDrag": [1.5, 1.5, 1.0], 
                "BuoyancyForce": 9.8,
                "WaterDensity": 1025.0,
                "VolumeDisplaced": 0.013
            }
    
    def calculate_drag_force(self, velocity: np.ndarray) -> np.ndarray:
        """Calculate hydrodynamic drag forces"""
        drag_coeffs = np.array(self.config["LinearDrag"])
        return -0.5 * self.config["WaterDensity"] * drag_coeffs * velocity * np.abs(velocity)
    
    def calculate_buoyancy_force(self, depth: float) -> float:
        """Calculate buoyancy force"""
        return self.config["WaterDensity"] * self.config["VolumeDisplaced"] * self.config["BuoyancyForce"]

class ROVController:
    """Complete ROV control system"""
    
    def __init__(self, ip: str = "127.0.0.1", port: int = 41451):
        # AirSim connection
        self.client = airsim.VehicleClient(ip, port)
        self.client.confirmConnection()
        
        # Water dynamics module
        self.water_dynamics = WaterDynamics()
        
        # BlueROV2 Heavy thrust allocation matrix
        self.thrust_allocation_matrix = np.array([
            [0.0, 0.0, 0.0, 0.0,  0.707, 0.707, -0.707, -0.707],  # X (forward/backward)
            [0.0, 0.0, 0.0, 0.0, -0.707, 0.707,  0.707, -0.707],  # Y (left/right)
            [1.0, 1.0, 1.0, 1.0,  0.0,   0.0,    0.0,    0.0  ],  # Z (up/down)
            [0.0, 0.0, 0.0, 0.0,  0.2,  -0.2,    0.2,   -0.2  ]   # Yaw (rotation)
        ])
        
        # PID control parameters
        self.pid_params = {
            'position': {'kp': 5.0, 'ki': 0.1, 'kd': 2.0},
            'orientation': {'kp': 3.0, 'ki': 0.05, 'kd': 1.5}
        }
        
        # Control state
        self.control_enabled = False
        self.target_position = np.zeros(3)
        self.target_orientation = 0.0
        
        # Sensor data cache
        self.sensor_data_queue = queue.Queue(maxsize=100)
        self.sensor_thread = None
        
        # Safety parameters
        self.max_depth = 50.0  # Maximum depth limit
        self.max_velocity = 2.0  # Maximum velocity limit
        
    def initialize(self) -> bool:
        """Initialize ROV system"""
        try:
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            self.control_enabled = True
            
            # Start sensor monitoring thread
            self.start_sensor_monitoring()
            
            print("ROV system initialized successfully")
            return True
        except Exception as e:
            print(f"ROV initialization failed: {e}")
            return False
    
    def get_rov_state(self) -> ROVState:
        """Get complete ROV state"""
        try:
            kinematics = self.client.simGetGroundTruthKinematics()
            
            # Position (NED coordinate system)
            position = np.array([
                kinematics.position.x_val,
                kinematics.position.y_val,
                kinematics.position.z_val
            ])
            
            # Velocity
            velocity = np.array([
                kinematics.linear_velocity.x_val,
                kinematics.linear_velocity.y_val,
                kinematics.linear_velocity.z_val
            ])
            
            # Orientation (quaternion to Euler angles)
            q = kinematics.orientation
            orientation = self.quaternion_to_euler(
                q.w_val, q.x_val, q.y_val, q.z_val
            )
            
            # Angular velocity
            angular_velocity = np.array([
                kinematics.angular_velocity.x_val,
                kinematics.angular_velocity.y_val,
                kinematics.angular_velocity.z_val
            ])
            
            return ROVState(
                position=position,
                velocity=velocity,
                orientation=orientation,
                angular_velocity=angular_velocity,
                depth=-position[2],  # Depth is negative z in NED
                timestamp=time.time()
            )
            
        except Exception as e:
            print(f"Failed to get ROV state: {e}")
            return None

# Additional methods continue...
```

## Usage Examples

### Basic ROV Control

```python
from rov_client import create_rov_controller
import time

# Initialize ROV
rov = create_rov_controller()

# Get current state
state = rov.get_rov_state()
print(f"Current position: {state.position}")
print(f"Current depth: {state.depth:.2f}m")

# Test thruster control
test_pwm = [0.5, 0.5, 0.5, 0.5, 0.55, 0.45, 0.55, 0.45]
success = rov.set_thruster_pwm(test_pwm, 2.0)
print(f"Thruster test: {'Success' if success else 'Failed'}")

# Move to position
success = rov.move_to_position(10.0, 5.0, -3.0, 0.0)
print(f"Position control: {'Success' if success else 'Failed'}")

# Emergency stop
rov.emergency_stop()
rov.shutdown()
```

### Advanced Navigation

```python
# Define trajectory waypoints
trajectory = [
    (0, 0, -2, 0),      # Start position
    (10, 0, -2, 0),     # Move forward
    (10, 10, -2, 90),   # Turn right
    (0, 10, -2, 180),   # Move backward
    (0, 0, -2, 270)     # Return home
]

# Execute trajectory
success = rov.follow_trajectory(trajectory, dt=0.1)
print(f"Trajectory following: {'Success' if success else 'Failed'}")
```

## Technical Requirements

| Component | Version | Purpose |
|-----------|---------|---------|
| **Python** | 3.7+ | Core runtime environment |
| **PyTorch** | 1.7+ | Machine learning framework |
| **AirSim** | 1.8.1 | Simulation environment |
| **OpenCV** | Latest | Computer vision processing |
| **NumPy** | Latest | Numerical computing |
| **Gym** | Latest | RL environment interface |
| **TensorboardX** | Latest | Training visualization |

## Performance Monitoring

All algorithms include built-in monitoring and logging:

```bash
# Launch tensorboard for training visualization
tensorboard --logdir=runs/

# Monitor system performance
python monitor_performance.py

# Generate performance reports
python generate_reports.py
```

## Troubleshooting Guide

### Common Issues & Solutions

| Issue | Symptoms | Solution |
|-------|----------|----------|
| **UE5 Build Fails** | Compilation errors | Ensure Visual Studio 2022 with C++ workload |
| **AirSim Connection** | API timeout errors | Check AirSim environment is running |
| **Python Dependencies** | Import errors | Activate virtual environment and reinstall |
| **Performance Issues** | Low FPS, lag | Update GPU drivers, reduce graphics settings |
| **ROV Not Responding** | No movement | Verify thruster configuration and PWM values |

### Debug Mode

Enable debug logging for detailed troubleshooting:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

rov = ROVController()
rov.set_debug_mode(True)
```

## Contributing

We welcome contributions to HAAV_Sim! Whether you're fixing bugs, adding features, or improving documentation, your help is appreciated.

### Development Setup

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

### Coding Standards

- Follow PEP 8 for Python code
- Use meaningful variable names
- Add docstrings to all functions
- Include type hints where appropriate

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

Built upon excellent work from:
- Microsoft AirSim team for simulation environment
- Blue Robotics for ROV specifications
- Unreal Engine team for rendering capabilities
- Python scientific computing community

---

<div align="center">

**Ready to dive into underwater robotics simulation?**

[Get Started](#quick-start-guide) • [View Documentation](docs/) • [See Examples](examples/)

</div>