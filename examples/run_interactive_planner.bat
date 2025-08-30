@echo off
REM ===============================================
REM 交互式轨迹规划器启动脚本
REM 提供图形界面进行轨迹设计和执行
REM ===============================================

setlocal enabledelayedexpansion
set PROJECT_ROOT=%~dp0..

echo.
echo ===============================================
echo    HAAV_Sim 交互式轨迹规划器
echo ===============================================
echo.

REM 检查Python环境
echo [信息] 检查Python环境...
if exist "%PROJECT_ROOT%\venv\Scripts\activate.bat" (
    echo [信息] 激活Python虚拟环境...
    call "%PROJECT_ROOT%\venv\Scripts\activate.bat"
) else (
    echo [警告] 未找到Python虚拟环境，使用系统Python
)

REM 检查必要依赖
echo [信息] 检查依赖包...
python -c "import tkinter, matplotlib" 2>nul
if %errorLevel% neq 0 (
    echo [错误] 缺少必要的GUI依赖包
    echo [解决] 请运行以下命令安装:
    echo   pip install matplotlib tkinter
    echo   或运行 setup\setup_environment.bat 重新配置环境
    pause
    exit /b 1
)

REM 检查程序文件
if not exist "%~dp0interactive_trajectory_planner.py" (
    echo [错误] 未找到交互式规划器程序文件
    pause
    exit /b 1
)

echo [成功] 环境检查通过
echo.

echo ===============================================
echo         交互式轨迹规划器功能介绍
echo ===============================================
echo.
echo  主要功能:
echo    图形化航点编辑
echo    实时轨迹预览
echo    多种轨迹类型 (直线、曲线、螺旋等)
echo    ROV连接和控制
echo    轨迹执行监控
echo    数据保存和加载
echo.
echo  使用方法:
echo   1. 点击"连接ROV"按钮连接到仿真环境
echo   2. 在左侧面板添加航点 (输入坐标或使用快速模板)
echo   3. 选择轨迹类型并生成完整轨迹
echo   4. 在3D视图中预览轨迹效果
echo   5. 执行轨迹并监控ROV运动
echo.
echo  提示:
echo   - 支持鼠标在3D图中交互操作
echo   - 可以保存轨迹文件供后续使用
echo   - 支持实时调整轨迹参数
echo.

REM 检查UE5是否运行 (可选)
echo [检查] 验证AirSim连接状态...
python -c "import airsim; client=airsim.VehicleClient(); client.confirmConnection(); print('[成功] AirSim连接正常')" 2>nul
if %errorLevel% neq 0 (
    echo [提示] 当前无法连接到AirSim
    echo [说明] 这不影响规划器启动，但无法执行实际轨迹
    echo [建议] 启动UE5项目后在规划器中点击"连接ROV"
    echo.
)

echo ===============================================
echo.
echo [信息] 即将启动交互式轨迹规划器...
echo [注意] 启动后将打开GUI窗口
echo [提示] 如需退出，请关闭GUI窗口或按Ctrl+C
echo.

REM 倒计时
echo 3秒后启动...
timeout /t 3 /nobreak >nul

echo [启动] 正在启动GUI应用程序...
echo.

REM 启动交互式规划器
python "%~dp0interactive_trajectory_planner.py"

set EXIT_CODE=%errorLevel%

echo.
echo ===============================================
if %EXIT_CODE% equ 0 (
    echo       程序正常退出
    echo ===============================================
    echo.
    echo [信息] 交互式轨迹规划器已关闭
    echo [数据] 如有保存文件，请查看当前目录
) else (
    echo       程序异常退出
    echo ===============================================
    echo.
    echo [错误] 程序执行过程中出现错误 (错误代码: %EXIT_CODE%)
    echo.
    echo [常见问题]:
    echo   1. GUI库未安装或版本不兼容
    echo   2. Python环境配置错误
    echo   3. 显示器分辨率或缩放问题
    echo   4. 权限不足或文件占用
    echo.
    echo [解决方案]:
    echo   - 检查Python环境: python --version
    echo   - 重新安装GUI依赖: pip install --upgrade matplotlib tkinter
    echo   - 运行环境诊断: %PROJECT_ROOT%\setup\diagnose_system.bat
    echo   - 以管理员身份重试
)

echo.
echo [相关工具]:
echo   - 基础轨迹演示: run_trajectory_demos.bat
echo   - 完整功能演示: complete_rov_demo.py
echo   - 系统诊断工具: %PROJECT_ROOT%\setup\diagnose_system.bat
echo.

pause