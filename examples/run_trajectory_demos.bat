@echo off
REM ===============================================
REM HAAV_Sim 轨迹演示运行脚本
REM 支持多种复杂轨迹的自动演示
REM ===============================================

setlocal enabledelayedexpansion
set PROJECT_ROOT=%~dp0..

echo.
echo ===============================================
echo    HAAV_Sim 高级轨迹演示程序
echo ===============================================
echo.

REM 检查Python环境
if exist "%PROJECT_ROOT%\venv\Scripts\activate.bat" (
    echo [信息] 激活Python虚拟环境...
    call "%PROJECT_ROOT%\venv\Scripts\activate.bat"
) else (
    echo [警告] 未找到Python虚拟环境，使用系统Python
)

REM 检查必要文件
if not exist "%~dp0trajectory_demos.py" (
    echo [错误] 未找到轨迹演示程序: trajectory_demos.py
    pause
    exit /b 1
)

echo [信息] 可用的轨迹演示:
echo   1. spiral_ascent    - 螺旋上升轨迹
echo   2. figure_eight     - 八字轨迹  
echo   3. sine_wave        - 正弦波轨迹
echo   4. helix_3d         - 3D螺旋轨迹
echo   5. lemniscate_3d    - 3D双纽线轨迹
echo   6. rose_curve       - 玫瑰线轨迹
echo   7. cloverleaf       - 四叶草轨迹
echo   8. complex_mission  - 复合轨迹任务
echo   9. all              - 运行所有演示
echo   0. preview          - 轨迹预览模式
echo.

REM 获取用户选择
if "%1"=="" (
    set /p choice="请选择要运行的演示 (输入数字或名称, 默认为all): "
    if "!choice!"=="" set choice=all
) else (
    set choice=%1
)

REM 映射数字到演示名称
if "%choice%"=="1" set demo_name=spiral_ascent
if "%choice%"=="2" set demo_name=figure_eight
if "%choice%"=="3" set demo_name=sine_wave
if "%choice%"=="4" set demo_name=helix_3d
if "%choice%"=="5" set demo_name=lemniscate_3d
if "%choice%"=="6" set demo_name=rose_curve
if "%choice%"=="7" set demo_name=cloverleaf
if "%choice%"=="8" set demo_name=complex_mission
if "%choice%"=="9" set demo_name=all
if "%choice%"=="0" set demo_name=preview

REM 如果没有映射到数字，直接使用输入的名称
if not defined demo_name set demo_name=%choice%

echo [信息] 即将运行演示: %demo_name%
echo.

REM 检查UE5项目是否运行
echo [检查] 验证UE5仿真环境...
python -c "import airsim; client=airsim.VehicleClient(); client.confirmConnection(); print('[成功] AirSim连接正常')" 2>nul
if %errorLevel% neq 0 (
    echo [警告] 无法连接到AirSim，请确保:
    echo   1. UE5项目正在运行
    echo   2. 项目加载完成并显示仿真场景
    echo   3. AirSim插件已正确加载
    echo.
    echo [提示] 可以运行以下命令启动仿真:
    echo   %PROJECT_ROOT%\launch_simulation.bat
    echo.
    set /p continue="是否继续尝试运行演示? (y/N): "
    if /i not "!continue!"=="y" (
        echo 演示已取消
        pause
        exit /b 0
    )
)

echo.
echo ===============================================
echo 开始运行轨迹演示: %demo_name%
echo ===============================================
echo.

REM 根据演示类型显示特殊说明
if "%demo_name%"=="all" (
    echo [信息] 完整演示模式 - 将依次运行所有8种轨迹
    echo [预计时间] 约15-20分钟
    echo [建议] 确保有足够时间完成所有演示
)

if "%demo_name%"=="complex_mission" (
    echo [信息] 复合任务模式 - 多阶段复杂轨迹任务
    echo [包含] 螺旋下降 → 八字巡航 → 玫瑰采样 → 直线返回
    echo [预计时间] 约2分钟
)

if "%demo_name%"=="preview" (
    echo [信息] 预览模式 - 仅生成轨迹图像，不执行实际控制
    echo [输出] 将生成各种轨迹的3D可视化图像
)

echo.
echo [提示] 演示期间可以按 Ctrl+C 中断程序
echo [注意] 请确保ROV周围有足够的空间进行轨迹运动
echo.

REM 倒计时
echo 5秒后开始演示...
timeout /t 5 /nobreak >nul

REM 运行轨迹演示程序
python "%~dp0trajectory_demos.py" --demo %demo_name% --visualize

set EXIT_CODE=%errorLevel%

echo.
echo ===============================================
if %EXIT_CODE% equ 0 (
    echo       轨迹演示完成!
    echo ===============================================
    echo.
    echo [成功] 演示 '%demo_name%' 执行完成
    echo [数据] 演示数据已保存到当前目录
    echo [图像] 轨迹可视化图像已生成 (如果适用)
    echo.
    
    REM 检查生成的文件
    echo [输出文件]:
    for %%f in (trajectory_*.json) do (
        echo   - %%f
    )
    for %%f in (*_preview.png) do (
        echo   - %%f  
    )
    
) else (
    echo       轨迹演示失败!
    echo ===============================================
    echo.
    echo [失败] 演示执行过程中出现错误 (错误代码: %EXIT_CODE%)
    echo [检查] 请查看上方的错误信息
    echo.
    echo [常见问题]:
    echo   1. UE5项目未运行或未完全加载
    echo   2. AirSim插件配置错误
    echo   3. Python环境缺少依赖包
    echo   4. ROV初始化失败
    echo.
    echo [解决方案]:
    echo   - 运行 %PROJECT_ROOT%\setup\diagnose_system.bat 进行诊断
    echo   - 检查 %PROJECT_ROOT%\PYTHON_USAGE.md 使用说明
    echo   - 查看详细错误日志并排除问题
)

echo.
echo [下一步]:
if "%demo_name%"=="preview" (
    echo   - 查看生成的轨迹预览图像
    echo   - 选择感兴趣的轨迹进行实际演示
    echo   - 运行: run_trajectory_demos.bat [演示名称]
) else (
    echo   - 分析保存的轨迹数据 (JSON格式)
    echo   - 尝试其他轨迹演示
    echo   - 自定义轨迹参数进行测试
)

echo   - 运行完整功能演示: complete_rov_demo.py
echo   - 查看API文档进行自定义开发
echo.

pause