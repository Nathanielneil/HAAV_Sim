@echo off
setlocal enabledelayedexpansion

echo.
echo ===============================================
echo    HAAV_Sim Windows 最小实现 - 项目构建
echo ===============================================
echo.

REM 设置路径变量
set PROJECT_ROOT=%~dp0..
set UE5_PATH=C:\Program Files\Epic Games\UE_5.1\Engine
set BUILD_DIR=%PROJECT_ROOT%\build
set BUILD_LOG=%PROJECT_ROOT%\setup\build_log.txt

REM 检查项目根目录
if not exist "%PROJECT_ROOT%" (
    echo [错误] 项目根目录不存在: %PROJECT_ROOT%
    pause
    exit /b 1
)

echo [信息] 项目根目录: %PROJECT_ROOT%
echo [信息] 构建日志: %BUILD_LOG%
echo. > %BUILD_LOG%

REM 1. 检查UE5安装
echo [步骤 1/8] 检查Unreal Engine 5安装...
if not exist "%UE5_PATH%" (
    echo [错误] 未找到UE5安装目录: %UE5_PATH%
    echo [解决] 请确认UE5已正确安装，或修改UE5_PATH变量
    echo.
    echo 常见UE5安装路径:
    echo   C:\Program Files\Epic Games\UE_5.1\Engine
    echo   C:\Program Files\Epic Games\UE_5.0\Engine
    echo   自定义安装路径
    echo.
    pause
    exit /b 1
)
echo [成功] UE5安装验证通过: %UE5_PATH%
echo.

REM 2. 创建构建目录结构
echo [步骤 2/8] 创建项目目录结构...
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
if not exist "%PROJECT_ROOT%\unreal" mkdir "%PROJECT_ROOT%\unreal"
if not exist "%PROJECT_ROOT%\src\cpp" mkdir "%PROJECT_ROOT%\src\cpp"
if not exist "%PROJECT_ROOT%\src\python" mkdir "%PROJECT_ROOT%\src\python"
if not exist "%PROJECT_ROOT%\config" mkdir "%PROJECT_ROOT%\config"
if not exist "%PROJECT_ROOT%\assets" mkdir "%PROJECT_ROOT%\assets"
if not exist "%PROJECT_ROOT%\docs" mkdir "%PROJECT_ROOT%\docs"

echo [成功] 目录结构创建完成
echo.

REM 3. 下载AirSim源码
echo [步骤 3/8] 获取AirSim源码...
if not exist "%BUILD_DIR%\AirSim" (
    echo [信息] 正在克隆AirSim源码...
    cd /d "%BUILD_DIR%"
    git clone --depth 1 https://github.com/microsoft/AirSim.git >> %BUILD_LOG% 2>&1
    
    if %errorLevel% neq 0 (
        echo [错误] AirSim源码下载失败，请检查网络连接
        echo [建议] 可以手动下载AirSim并解压到 %BUILD_DIR%\AirSim
        pause
        exit /b 1
    )
    
    echo [成功] AirSim源码下载完成
) else (
    echo [信息] AirSim源码已存在，跳过下载
)
echo.

REM 4. 构建AirSim库
echo [步骤 4/8] 构建AirSim库...
cd /d "%BUILD_DIR%\AirSim"

if not exist "build.cmd" (
    echo [错误] 未找到AirSim构建脚本
    pause
    exit /b 1
)

echo [信息] 正在编译AirSim (这可能需要15-30分钟)...
call build.cmd >> %BUILD_LOG% 2>&1

if %errorLevel% neq 0 (
    echo [错误] AirSim编译失败，请查看日志: %BUILD_LOG%
    echo [常见问题]
    echo   - 确认Visual Studio Build Tools已安装
    echo   - 确认有足够的磁盘空间 (至少5GB)
    echo   - 尝试以管理员身份运行
    pause
    exit /b 1
)

echo [成功] AirSim库编译完成
echo.

REM 5. 创建UE5项目
echo [步骤 5/8] 创建UE5项目...
cd /d "%PROJECT_ROOT%\unreal"

if not exist "Underwater.uproject" (
    echo [信息] 创建UE5项目文件...
    (
    echo {
    echo     "FileVersion": 3,
    echo     "EngineAssociation": "5.1",
    echo     "Category": "",
    echo     "Description": "HAAV_Sim Windows Minimal - Underwater ROV Simulation",
    echo     "Modules": [
    echo         {
    echo             "Name": "Underwater",
    echo             "Type": "Runtime",
    echo             "LoadingPhase": "Default",
    echo             "AdditionalDependencies": [
    echo                 "Engine",
    echo                 "CoreUObject",
    echo                 "AirSim"
    echo             ]
    echo         }
    echo     ],
    echo     "Plugins": [
    echo         {
    echo             "Name": "AirSim",
    echo             "Enabled": true
    echo         },
    echo         {
    echo             "Name": "Water",
    echo             "Enabled": true
    echo         }
    echo     ]
    echo }
    ) > Underwater.uproject
    
    echo [成功] UE5项目文件创建完成
) else (
    echo [信息] UE5项目文件已存在
)
echo.

REM 6. 复制AirSim插件
echo [步骤 6/8] 安装AirSim插件...
if not exist "Plugins" mkdir "Plugins"
if not exist "Plugins\AirSim" (
    echo [信息] 复制AirSim插件...
    xcopy "%BUILD_DIR%\AirSim\Unreal\Plugins\AirSim" "Plugins\AirSim" /E /I /H /Y >> %BUILD_LOG% 2>&1
    
    if %errorLevel% neq 0 (
        echo [错误] AirSim插件复制失败
        pause
        exit /b 1
    )
    
    echo [成功] AirSim插件安装完成
) else (
    echo [信息] AirSim插件已存在
)
echo.

REM 7. 生成项目文件
echo [步骤 7/8] 生成Visual Studio项目文件...
echo [信息] 正在生成项目文件 (可能需要几分钟)...

"%UE5_PATH%\Binaries\DotNET\UnrealBuildTool.exe" -projectfiles -project="%PROJECT_ROOT%\unreal\Underwater.uproject" -game -rocket -progress >> %BUILD_LOG% 2>&1

if %errorLevel% neq 0 (
    echo [错误] 项目文件生成失败
    echo [建议] 检查UE5路径是否正确，或尝试手动打开.uproject文件
    pause
    exit /b 1
)

echo [成功] Visual Studio项目文件生成完成
echo.

REM 8. 编译UE5项目
echo [步骤 8/8] 编译UE5项目...
echo [信息] 正在编译项目 (这可能需要20-40分钟)...

"%UE5_PATH%\Binaries\DotNET\UnrealBuildTool.exe" UnderwaterEditor Win64 Development "%PROJECT_ROOT%\unreal\Underwater.uproject" -waitmutex >> %BUILD_LOG% 2>&1

if %errorLevel% neq 0 (
    echo [警告] 项目编译出现问题，但项目文件已生成
    echo [建议] 可以手动在Visual Studio中打开并编译项目
) else (
    echo [成功] UE5项目编译完成
)
echo.

REM 9. 复制默认配置文件
echo [信息] 复制默认配置文件...
if not exist "%USERPROFILE%\Documents\AirSim" mkdir "%USERPROFILE%\Documents\AirSim"

REM 创建默认settings.json
if not exist "%USERPROFILE%\Documents\AirSim\settings.json" (
    (
    echo {
    echo   "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
    echo   "SettingsVersion": 1.2,
    echo   "SimMode": "Rov",
    echo   "ClockSpeed": 1,
    echo   "EnableCollisions": true,
    echo   "PawnPaths": {
    echo     "DefaultRov": {"PawnBP": "Class'/AirSim/Blueprints/BP_RovPawn.BP_RovPawn_C'"}
    echo   },
    echo   "Vehicles": {
    echo     "BlueROV": {
    echo       "VehicleType": "RovSimple",
    echo       "DefaultVehicleState": "Armed",
    echo       "PawnPath": "DefaultRov",
    echo       "EnableCollisions": true,
    echo       "AllowAPIAlways": true,
    echo       "Cameras": {
    echo         "front_center": {
    echo           "CaptureSettings": [{"ImageType": 0, "Width": 1920, "Height": 1080, "FOV_Degrees": 90}],
    echo           "X": 0.30, "Y": 0.0, "Z": 0.0
    echo         }
    echo       }
    echo     }
    echo   }
    echo }
    ) > "%USERPROFILE%\Documents\AirSim\settings.json"
    
    echo [成功] 默认AirSim配置已创建
)

REM 10. 创建项目启动脚本
echo [信息] 创建项目启动脚本...
(
echo @echo off
echo echo 启动HAAV_Sim水下仿真环境...
echo echo.
echo if exist "%PROJECT_ROOT%\venv\Scripts\activate.bat" ^(
echo     call "%PROJECT_ROOT%\venv\Scripts\activate.bat"
echo     echo [信息] Python虚拟环境已激活
echo ^)
echo echo.
echo echo [信息] 启动UE5编辑器...
echo start "" "%PROJECT_ROOT%\unreal\Underwater.uproject"
echo echo.
echo echo [信息] 等待UE5启动完成后，可以运行以下Python程序:
echo echo   python "%PROJECT_ROOT%\examples\complete_rov_demo.py"
echo echo.
echo pause
) > "%PROJECT_ROOT%\launch_simulation.bat"

echo [成功] 项目启动脚本已创建: launch_simulation.bat
echo.

REM 构建完成报告
echo.
echo ===============================================
echo              构建完成报告
echo ===============================================
echo.
echo [✓] AirSim库编译完成
echo [✓] UE5项目创建完成  
echo [✓] AirSim插件安装完成
echo [✓] Visual Studio项目生成完成
echo [✓] 默认配置文件创建完成
echo [✓] 项目启动脚本创建完成
echo.
echo [项目文件]
echo   - UE5项目: %PROJECT_ROOT%\unreal\Underwater.uproject
echo   - AirSim配置: %USERPROFILE%\Documents\AirSim\settings.json
echo   - 启动脚本: %PROJECT_ROOT%\launch_simulation.bat
echo.
echo [下一步]
echo   1. 运行 setup_environment.bat 配置Python环境
echo   2. 运行 launch_simulation.bat 启动仿真
echo   3. 运行 examples\complete_rov_demo.py 测试功能
echo.
echo [注意事项]
echo   - 首次启动UE5可能需要编译着色器 (5-15分钟)
echo   - 确保显卡驱动程序是最新版本
echo   - 建议关闭其他大型程序以节省内存
echo.
echo ===============================================

echo 构建日志已保存到: %BUILD_LOG%
echo.
pause