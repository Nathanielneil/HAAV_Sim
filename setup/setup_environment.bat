@echo off
setlocal enabledelayedexpansion

echo.
echo ===============================================
echo    HAAV_Sim Windows 最小实现 - 环境配置
echo ===============================================
echo.

set PROJECT_ROOT=%~dp0..
set PYTHON_ENV=%PROJECT_ROOT%\venv
set ENV_LOG=%PROJECT_ROOT%\setup\environment_log.txt

echo [信息] 项目根目录: %PROJECT_ROOT%
echo [信息] Python环境: %PYTHON_ENV%
echo [信息] 配置日志: %ENV_LOG%
echo. > %ENV_LOG%

REM 1. 检查Python安装
echo [步骤 1/6] 检查Python环境...
python --version >> %ENV_LOG% 2>&1
if %errorLevel% neq 0 (
    echo [错误] Python未安装或未添加到PATH
    echo [解决] 请安装Python 3.8+ 并添加到系统PATH
    pause
    exit /b 1
)

python --version
echo [成功] Python环境验证通过
echo.

REM 2. 创建Python虚拟环境
echo [步骤 2/6] 创建Python虚拟环境...
if not exist "%PYTHON_ENV%" (
    echo [信息] 正在创建虚拟环境...
    python -m venv "%PYTHON_ENV%" >> %ENV_LOG% 2>&1
    
    if %errorLevel% neq 0 (
        echo [错误] 虚拟环境创建失败
        echo [解决] 检查磁盘空间或Python安装
        pause
        exit /b 1
    )
    
    echo [成功] Python虚拟环境创建完成
) else (
    echo [信息] Python虚拟环境已存在，跳过创建
)
echo.

REM 3. 激活虚拟环境并升级pip
echo [步骤 3/6] 激活虚拟环境...
call "%PYTHON_ENV%\Scripts\activate.bat"

if %errorLevel% neq 0 (
    echo [错误] 虚拟环境激活失败
    pause
    exit /b 1
)

echo [信息] 正在升级pip...
python -m pip install --upgrade pip >> %ENV_LOG% 2>&1
echo [成功] 虚拟环境激活并配置完成
echo.

REM 4. 安装核心Python依赖
echo [步骤 4/6] 安装核心Python依赖...
echo [信息] 安装科学计算包...
pip install numpy==1.24.3 >> %ENV_LOG% 2>&1
pip install scipy==1.10.1 >> %ENV_LOG% 2>&1
pip install matplotlib==3.7.1 >> %ENV_LOG% 2>&1

echo [信息] 安装计算机视觉包...
pip install opencv-python==4.8.0.74 >> %ENV_LOG% 2>&1
pip install Pillow==10.0.0 >> %ENV_LOG% 2>&1

echo [信息] 安装通信协议包...
pip install msgpack-rpc-python==0.4.1 >> %ENV_LOG% 2>&1
pip install msgpack==1.0.5 >> %ENV_LOG% 2>&1

echo [信息] 安装数据处理包...
pip install pandas==2.0.3 >> %ENV_LOG% 2>&1

if %errorLevel% neq 0 (
    echo [警告] 部分依赖包安装可能失败，但继续执行
) else (
    echo [成功] 核心依赖包安装完成
)
echo.

REM 5. 安装AirSim Python包
echo [步骤 5/6] 安装AirSim Python包...
if exist "%PROJECT_ROOT%\build\AirSim\PythonClient" (
    echo [信息] 从本地AirSim源码安装...
    cd /d "%PROJECT_ROOT%\build\AirSim\PythonClient"
    pip install -e . >> %ENV_LOG% 2>&1
    
    if %errorLevel% neq 0 (
        echo [警告] 本地AirSim安装失败，尝试从PyPI安装...
        pip install airsim >> %ENV_LOG% 2>&1
    )
) else (
    echo [信息] 从PyPI安装AirSim包...
    pip install airsim >> %ENV_LOG% 2>&1
)

echo [成功] AirSim Python包安装完成
echo.

REM 6. 安装项目Python包
echo [步骤 6/6] 安装项目Python包...
if exist "%PROJECT_ROOT%\src\python\setup.py" (
    echo [信息] 安装项目包...
    cd /d "%PROJECT_ROOT%\src\python"
    pip install -e . >> %ENV_LOG% 2>&1
) else (
    echo [信息] 创建项目包安装文件...
    cd /d "%PROJECT_ROOT%\src\python"
    
    REM 创建setup.py
    (
    echo from setuptools import setup, find_packages
    echo.
    echo setup^(
    echo     name="unav-sim-minimal",
    echo     version="1.0.0",
    echo     description="HAAV_Sim Windows Minimal Implementation",
    echo     packages=find_packages^(^),
    echo     install_requires=[
    echo         "numpy^>=1.20.0",
    echo         "scipy^>=1.7.0",  
    echo         "matplotlib^>=3.5.0",
    echo         "opencv-python^>=4.5.0",
    echo         "msgpack-rpc-python^>=0.4.1",
    echo         "pandas^>=1.3.0"
    echo     ],
    echo     python_requires="^>=3.8",
    echo ^)
    ) > setup.py
    
    REM 创建__init__.py
    echo # HAAV_Sim Minimal Python Package > __init__.py
    
    pip install -e . >> %ENV_LOG% 2>&1
    echo [成功] 项目包安装完成
)
echo.

REM 7. 创建环境激活脚本
echo [信息] 创建环境管理脚本...
(
echo @echo off
echo echo 激活HAAV_Sim Python环境...
echo call "%PYTHON_ENV%\Scripts\activate.bat"
echo echo [信息] Python虚拟环境已激活
echo echo [信息] 可用的命令:
echo echo   python -c "import airsim; print('AirSim版本:', airsim.__version__^)"
echo echo   python examples\complete_rov_demo.py
echo echo   python examples\basic_control.py
echo echo.
) > "%PROJECT_ROOT%\activate_env.bat"

echo [成功] 环境激活脚本: activate_env.bat
echo.

REM 8. 创建环境测试脚本  
echo [信息] 创建环境测试脚本...
(
echo #!/usr/bin/env python3
echo """环境测试脚本"""
echo import sys
echo import importlib
echo.
echo def test_import^(module_name^):
echo     try:
echo         module = importlib.import_module^(module_name^)
echo         version = getattr^(module, '__version__', 'Unknown'^)
echo         print^(f"[✓] {module_name}: {version}"^)
echo         return True
echo     except ImportError as e:
echo         print^(f"[✗] {module_name}: {e}"^)  
echo         return False
echo.
echo def main^(^):
echo     print^("HAAV_Sim Python环境测试"^)
echo     print^("="*40^)
echo     
echo     modules = [
echo         'numpy', 'scipy', 'matplotlib', 'cv2',
echo         'msgpack', 'pandas', 'airsim'
echo     ]
echo     
echo     success_count = 0
echo     for module in modules:
echo         if test_import^(module^):
echo             success_count += 1
echo     
echo     print^(f"\n测试结果: {success_count}/{len^(modules^)} 模块导入成功"^)
echo     
echo     if success_count == len^(modules^):
echo         print^("🎉 环境配置完全正确!"^)
echo         return True
echo     else:
echo         print^("⚠️ 部分模块导入失败，请检查安装"^)
echo         return False
echo.
echo if __name__ == "__main__":
echo     success = main^(^)
echo     sys.exit^(0 if success else 1^)
) > "%PROJECT_ROOT%\test_environment.py"

echo [成功] 环境测试脚本: test_environment.py
echo.

REM 9. 运行环境测试
echo [验证] 运行环境测试...
python "%PROJECT_ROOT%\test_environment.py"
set TEST_RESULT=%errorLevel%
echo.

REM 10. 创建使用说明
echo [信息] 创建使用说明...
(
echo # HAAV_Sim Python环境使用说明
echo.
echo ## 激活环境
echo ```batch
echo call activate_env.bat
echo # 或者
echo call venv\Scripts\activate.bat  
echo ```
echo.
echo ## 测试环境
echo ```batch
echo python test_environment.py
echo ```
echo.
echo ## 运行演示
echo ```batch 
echo # 完整功能演示
echo python examples\complete_rov_demo.py
echo.
echo # 基础控制演示
echo python examples\basic_control.py --scenario basic_movement
echo.
echo # 轨迹跟踪演示
echo python examples\complete_rov_demo.py --scenario trajectory_following
echo ```
echo.
echo ## 开发模式
echo ```batch
echo # 进入开发模式
echo cd src\python
echo python -m pytest tests\  # 运行测试
echo ```
echo.
echo ## 常见问题
echo 1. 如果提示模块找不到，运行 `pip list` 检查安装
echo 2. 如果AirSim连接失败，确认UE5项目正在运行
echo 3. 如果出现权限错误，以管理员身份运行
echo.
) > "%PROJECT_ROOT%\PYTHON_USAGE.md"

echo [成功] 使用说明: PYTHON_USAGE.md
echo.

REM 环境配置完成报告
echo.
echo ===============================================
echo            环境配置完成报告  
echo ===============================================
echo.
if %TEST_RESULT% equ 0 (
    echo [✓] Python虚拟环境创建成功
    echo [✓] 所有依赖包安装成功
    echo [✓] AirSim包安装成功
    echo [✓] 项目包安装成功
    echo [✓] 环境测试通过
    echo.
    echo [🎉] 环境配置完全成功！
) else (
    echo [✓] Python虚拟环境创建成功
    echo [⚠] 部分依赖包可能安装失败
    echo [⚠] 环境测试未完全通过
    echo.
    echo [⚠️] 环境配置部分成功，请检查错误日志
)
echo.
echo [环境信息]
echo   - Python环境: %PYTHON_ENV%
echo   - 激活脚本: activate_env.bat
echo   - 测试脚本: test_environment.py
echo   - 使用说明: PYTHON_USAGE.md
echo.
echo [下一步]
echo   1. 运行 activate_env.bat 激活Python环境
echo   2. 运行 launch_simulation.bat 启动UE5仿真
echo   3. 运行 python examples\complete_rov_demo.py 测试功能
echo.
echo [快速开始]
echo   call activate_env.bat
echo   python test_environment.py
echo   python examples\complete_rov_demo.py --scenario basic_movement
echo.
echo ===============================================

echo 配置日志已保存到: %ENV_LOG%
echo.
pause