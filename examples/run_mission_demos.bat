@echo off
REM ===============================================
REM 高级任务演示运行脚本
REM 支持多种实用ROV任务场景的演示
REM ===============================================

setlocal enabledelayedexpansion
set PROJECT_ROOT=%~dp0..

echo.
echo ===============================================
echo    HAAV_Sim 高级任务演示程序
echo ===============================================
echo.

REM 检查Python环境
if exist "%PROJECT_ROOT%\venv\Scripts\activate.bat" (
    echo [信息] 激活Python虚拟环境...
    call "%PROJECT_ROOT%\venv\Scripts\activate.bat"
) else (
    echo [警告] 未找到Python虚拟环境，使用系统Python
)

REM 检查程序文件
if not exist "%~dp0advanced_mission_demos.py" (
    echo [错误] 未找到高级任务演示程序
    pause
    exit /b 1
)

echo [信息] 可用的高级任务演示:
echo.
echo   🔍 1. search_rescue         - 搜索救援任务
echo      描述: ROV搜索指定区域内的目标物体
echo      特点: 网格搜索模式, 目标识别, 详细检查
echo      时长: 约15分钟
echo.
echo   🏗️ 2. underwater_inspection - 水下结构检查
echo      描述: ROV检查水下结构的完整性和安全性  
echo      特点: 多角度检查, 缺陷识别, 质量评估
echo      时长: 约12分钟
echo.
echo   🧪 3. scientific_sampling   - 科学采样任务
echo      描述: ROV在指定位置采集水样和沉积物样本
echo      特点: 精确定位, 样本采集, 环境记录
echo      时长: 约10分钟
echo.
echo   🚰 4. pipeline_following    - 管道跟踪检查
echo      描述: ROV跟踪管道路径并检查异常情况
echo      特点: 路径跟踪, 异常检测, 连续监控
echo      时长: 约8分钟
echo.
echo   🌊 5. environmental_monitoring - 环境监控任务
echo      描述: ROV监控水质参数和环境指标
echo      特点: 多参数监测, 数据分析, 趋势评估
echo      时长: 约10分钟
echo.
echo   👁️ 6. surveillance_patrol   - 监视巡逻任务
echo      描述: ROV按预定路线进行安全巡逻监控
echo      特点: 路线巡逻, 事件检测, 图像记录
echo      时长: 约12分钟
echo.
echo   🎖️ 0. all                   - 运行所有任务演示
echo      描述: 依次执行所有6种高级任务演示
echo      时长: 约70分钟 (包含任务间隔)
echo.

REM 获取用户选择
if "%1"=="" (
    set /p choice="请选择要运行的任务演示 (输入数字或名称, 默认为all): "
    if "!choice!"=="" set choice=all
) else (
    set choice=%1
)

REM 映射数字到任务名称
if "%choice%"=="1" set mission_name=search_rescue
if "%choice%"=="2" set mission_name=underwater_inspection
if "%choice%"=="3" set mission_name=scientific_sampling
if "%choice%"=="4" set mission_name=pipeline_following
if "%choice%"=="5" set mission_name=environmental_monitoring
if "%choice%"=="6" set mission_name=surveillance_patrol
if "%choice%"=="0" set mission_name=all

REM 如果没有映射到数字，直接使用输入的名称
if not defined mission_name set mission_name=%choice%

echo [信息] 即将运行任务演示: %mission_name%
echo.

REM 根据任务类型显示详细说明
if "%mission_name%"=="search_rescue" (
    echo ===============================================
    echo           搜索救援任务演示
    echo ===============================================
    echo.
    echo 🎯 任务目标: 在指定水域搜索失踪人员或物体
    echo 📋 执行步骤:
    echo    1. 根据搜索区域生成网格搜索模式
    echo    2. ROV按蛇形路径覆盖整个搜索区域
    echo    3. 在每个搜索点悬停并进行全方位扫描
    echo    4. 使用视觉识别算法检测目标物体
    echo    5. 对发现的目标进行详细多角度检查
    echo    6. 记录目标位置、特征和置信度
    echo.
    echo 💡 技术亮点: 
    echo    - 智能搜索路径规划
    echo    - 实时目标检测算法
    echo    - 自适应搜索策略
    echo.
) else if "%mission_name%"=="underwater_inspection" (
    echo ===============================================
    echo         水下结构检查任务演示
    echo ===============================================
    echo.
    echo 🎯 任务目标: 检查水下建筑结构的完整性和安全状态
    echo 📋 执行步骤:
    echo    1. 围绕检查对象规划多角度检查轨迹
    echo    2. 从不同角度对结构进行详细观测
    echo    3. 使用计算机视觉识别结构缺陷
    echo    4. 评估缺陷的严重程度和影响
    echo    5. 生成详细的检查报告
    echo.
    echo 💡 技术亮点:
    echo    - 多角度检查策略
    echo    - 自动缺陷识别
    echo    - 质量评估算法
) else if "%mission_name%"=="scientific_sampling" (
    echo ===============================================
    echo         科学采样任务演示
    echo ===============================================
    echo.
    echo 🎯 任务目标: 在特定位置采集水样和沉积物用于科学研究
    echo 📋 执行步骤:
    echo    1. 精确导航到采样位置
    echo    2. 稳定悬停并启动采样设备
    echo    3. 根据采样类型执行相应采样程序
    echo    4. 记录环境参数和采样条件
    echo    5. 评估样本质量和采样效果
    echo.
    echo 💡 技术亮点:
    echo    - 精确位置控制
    echo    - 多类型采样支持
    echo    - 环境参数监控
) else if "%mission_name%"=="all" (
    echo ===============================================
    echo         完整任务演示套件
    echo ===============================================
    echo.
    echo 🎯 任务目标: 展示ROV在各种实际应用场景中的能力
    echo 📋 执行计划:
    echo    第1阶段: 搜索救援任务 (15分钟)
    echo    第2阶段: 水下结构检查 (12分钟) 
    echo    第3阶段: 科学采样任务 (10分钟)
    echo    第4阶段: 管道跟踪检查 (8分钟)
    echo    第5阶段: 环境监控任务 (10分钟)
    echo    第6阶段: 监视巡逻任务 (12分钟)
    echo.
    echo ⏱️ 总预计时间: 约70分钟 (包含任务间隔)
    echo 💡 建议: 确保有充足的时间完成所有演示
)

echo.

REM 检查AirSim连接状态
echo [检查] 验证AirSim连接状态...
python -c "import airsim; client=airsim.VehicleClient(); client.confirmConnection(); print('[成功] AirSim连接正常')" 2>nul
if %errorLevel% neq 0 (
    echo [警告] 当前无法连接到AirSim
    echo.
    echo [必需] 高级任务演示需要AirSim运行，请:
    echo   1. 启动UE5项目: %PROJECT_ROOT%\launch_simulation.bat
    echo   2. 等待项目完全加载并显示仿真场景
    echo   3. 确认AirSim插件正常工作
    echo.
    set /p continue="AirSim未运行，是否继续? (将会失败) (y/N): "
    if /i not "!continue!"=="y" (
        echo 演示已取消，请先启动AirSim
        pause
        exit /b 0
    )
)

echo.
echo ===============================================
echo 开始运行高级任务演示: %mission_name%
echo ===============================================
echo.

echo [提示] 任务演示特点:
echo   - 🤖 完全自主执行，无需人工干预
echo   - 📊 实时显示任务进度和状态
echo   - 💾 自动保存任务数据和结果
echo   - 🎥 记录关键操作和发现
echo   - 📈 生成详细的任务报告
echo.
echo [注意事项]:
echo   - 演示期间可以按 Ctrl+C 安全中断
echo   - ROV将自主执行复杂的任务流程
echo   - 请观察UE5窗口中的ROV运动
echo   - 任务数据将保存到当前目录
echo.

REM 倒计时
echo 5秒后开始任务演示...
timeout /t 5 /nobreak >nul

echo [启动] 正在启动高级任务演示程序...
echo.

REM 运行高级任务演示
python "%~dp0advanced_mission_demos.py" --mission %mission_name%

set EXIT_CODE=%errorLevel%

echo.
echo ===============================================
if %EXIT_CODE% equ 0 (
    echo        任务演示完成!
    echo ===============================================
    echo.
    echo [成功] 高级任务演示 '%mission_name%' 执行完成
    echo.
    
    echo [输出文件] 以下文件已生成:
    echo   📊 任务数据文件:
    for %%f in (*_mission_*.json) do (
        echo     - %%f
    )
    echo   📷 图像文件:
    for %%f in (target_*.png surveillance_*.png) do (
        echo     - %%f
    )
    if not exist "*_mission_*.json" (
        echo     (无任务数据文件生成)
    )
    
    echo.
    echo [数据分析] 可以使用以下方式分析任务数据:
    echo   - 使用JSON查看器打开任务数据文件
    echo   - 导入到Excel或其他数据分析软件
    echo   - 使用Python脚本进行深度分析
    echo.
    echo [后续建议]:
    echo   - 尝试运行其他任务演示了解不同场景
    echo   - 根据需要调整任务参数进行测试
    echo   - 开发自定义任务脚本
    
) else (
    echo        任务演示失败!
    echo ===============================================
    echo.
    echo [失败] 任务演示执行过程中出现错误 (错误代码: %EXIT_CODE%)
    echo.
    echo [常见问题]:
    echo   1. AirSim未运行或连接失败
    echo   2. ROV初始化失败
    echo   3. Python环境或依赖包问题
    echo   4. 仿真环境配置错误
    echo   5. 任务执行过程中的异常
    echo.
    echo [解决方案]:
    echo   检查AirSim状态:
    echo   - 确认UE5项目正在运行
    echo   - 检查AirSim插件是否正确加载
    echo   - 验证ROV模型是否正确显示
    echo.
    echo   检查Python环境:
    echo   - 运行: python test_environment.py
    echo   - 检查依赖包: pip list
    echo   - 重新配置: setup\setup_environment.bat
    echo.
    echo   系统诊断:
    echo   - 运行: %PROJECT_ROOT%\setup\diagnose_system.bat
    echo   - 查看详细日志信息
)

echo.
echo [相关演示]:
echo   - 基础控制演示: complete_rov_demo.py
echo   - 轨迹演示程序: run_trajectory_demos.bat  
echo   - 交互式规划器: run_interactive_planner.bat
echo.

pause