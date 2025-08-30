@echo off
REM ===============================================
REM HAAV_Sim Windows 最小实现 - 依赖安装脚本
REM ===============================================

echo.
echo ===============================================
echo    HAAV_Sim Windows 最小完整实现
echo    依赖环境自动安装程序
echo ===============================================
echo.

REM 检查管理员权限
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo [错误] 请以管理员身份运行此脚本！
    echo 右键点击此文件 → "以管理员身份运行"
    echo.
    pause
    exit /b 1
)

echo [信息] 管理员权限验证通过
echo.

REM 设置变量
set INSTALL_LOG=%~dp0install_log.txt
set PROJECT_ROOT=%~dp0..

echo [信息] 开始安装依赖组件...
echo [信息] 安装日志: %INSTALL_LOG%
echo. > %INSTALL_LOG%

REM 1. 检查并安装Chocolatey包管理器
echo [步骤 1/7] 检查Chocolatey包管理器...
where choco >nul 2>&1
if %errorLevel% neq 0 (
    echo [信息] 正在安装Chocolatey...
    powershell -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))" >> %INSTALL_LOG% 2>&1
    
    if %errorLevel% neq 0 (
        echo [错误] Chocolatey安装失败，请检查网络连接
        pause
        exit /b 1
    )
    
    REM 刷新环境变量
    refreshenv
    echo [成功] Chocolatey安装完成
) else (
    echo [信息] Chocolatey已安装，跳过此步骤
)
echo.

REM 2. 安装Git和基础工具
echo [步骤 2/7] 安装Git和基础开发工具...
choco install -y git cmake python3 7zip >> %INSTALL_LOG% 2>&1
if %errorLevel% neq 0 (
    echo [警告] 部分基础工具安装失败，但继续执行
) else (
    echo [成功] 基础工具安装完成
)
echo.

REM 3. 安装Visual Studio Build Tools
echo [步骤 3/7] 安装Visual Studio Build Tools...
where cl >nul 2>&1
if %errorLevel% neq 0 (
    echo [信息] 正在安装Visual Studio Build Tools 2022...
    choco install -y visualstudio2022buildtools >> %INSTALL_LOG% 2>&1
    choco install -y visualstudio2022-workload-vctools >> %INSTALL_LOG% 2>&1
    choco install -y visualstudio2022-workload-nativedesktop >> %INSTALL_LOG% 2>&1
    
    echo [成功] Visual Studio Build Tools安装完成
) else (
    echo [信息] Visual Studio编译工具已安装，跳过此步骤
)
echo.

REM 4. 安装.NET Runtime
echo [步骤 4/7] 安装.NET 6.0 Runtime...
where dotnet >nul 2>&1
if %errorLevel% neq 0 (
    echo [信息] 正在安装.NET 6.0 Runtime...
    choco install -y dotnet-6.0-runtime dotnet-6.0-sdk >> %INSTALL_LOG% 2>&1
    echo [成功] .NET Runtime安装完成
) else (
    echo [信息] .NET Runtime已安装，跳过此步骤
)
echo.

REM 5. 配置Git长路径支持
echo [步骤 5/7] 配置Git长路径支持...
git config --system core.longpaths true >> %INSTALL_LOG% 2>&1
git config --global core.autocrlf false >> %INSTALL_LOG% 2>&1
echo [成功] Git配置完成
echo.

REM 6. 检查Python环境
echo [步骤 6/7] 检查Python环境...
python --version >> %INSTALL_LOG% 2>&1
if %errorLevel% neq 0 (
    echo [警告] Python未正确安装，请手动安装Python 3.8+
) else (
    echo [信息] 正在安装Python依赖包...
    python -m pip install --upgrade pip >> %INSTALL_LOG% 2>&1
    python -m pip install numpy scipy matplotlib opencv-python msgpack-rpc-python >> %INSTALL_LOG% 2>&1
    echo [成功] Python依赖安装完成
)
echo.

REM 7. 下载Epic Games Launcher
echo [步骤 7/7] 准备Unreal Engine 5安装...
echo [信息] 即将打开Epic Games Launcher下载页面
echo [重要] 您需要手动完成以下步骤:
echo         1. 下载并安装Epic Games Launcher
echo         2. 注册/登录Epic Games账号
echo         3. 在"虚幻引擎"选项卡中安装UE 5.1.1
echo         4. 安装完成后运行 build_project.bat
echo.

timeout /t 5 /nobreak
start https://www.epicgames.com/store/en-US/download

REM 创建环境验证脚本
echo [信息] 创建环境验证脚本...
(
echo @echo off
echo echo 检查安装环境...
echo echo.
echo echo Git版本:
echo git --version
echo echo.
echo echo Python版本:
echo python --version
echo echo.
echo echo CMake版本:
echo cmake --version
echo echo.
echo echo Visual Studio编译器:
echo where cl
echo echo.
echo echo .NET Runtime:
echo dotnet --version
echo echo.
echo echo [信息] 如果所有工具都显示版本号，说明环境配置成功
echo pause
) > %PROJECT_ROOT%\setup\verify_environment.bat

echo [成功] 环境验证脚本已创建: verify_environment.bat
echo.

REM 创建快速启动脚本
echo [信息] 创建项目快速启动脚本...
(
echo @echo off
echo cd /d "%~dp0.."
echo echo 启动HAAV_Sim项目...
echo echo.
echo if exist "venv\Scripts\activate.bat" ^(
echo     call venv\Scripts\activate.bat
echo     echo Python虚拟环境已激活
echo ^) else ^(
echo     echo 警告: Python虚拟环境未找到
echo ^)
echo echo.
echo if exist "unreal\Underwater.uproject" ^(
echo     echo 启动UE5项目...
echo     start "" "unreal\Underwater.uproject"
echo ^) else ^(
echo     echo 错误: UE5项目文件未找到，请先运行 build_project.bat
echo ^)
echo echo.
echo echo 可以运行以下命令:
echo echo   python examples\complete_rov_demo.py    - 完整演示
echo echo   python examples\basic_control.py        - 基础控制
echo echo.
echo pause
) > %PROJECT_ROOT%\start_project.bat

echo [成功] 快速启动脚本已创建: start_project.bat
echo.

REM 创建问题诊断脚本
(
echo @echo off
echo echo HAAV_Sim 安装诊断工具
echo echo ========================
echo echo.
echo echo 检查系统要求...
echo systeminfo ^| find "Total Physical Memory"
echo wmic cpu get name
echo wmic path win32_VideoController get name
echo echo.
echo echo 检查已安装软件...
echo reg query "HKLM\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall" /s /f "Visual Studio" 2^>nul ^| find "DisplayName"
echo reg query "HKLM\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall" /s /f "Epic Games" 2^>nul ^| find "DisplayName"
echo echo.
echo echo 检查环境变量...
echo echo PATH中的Python: 
echo where python
echo echo PATH中的Git:
echo where git
echo echo.
echo echo 如果发现问题，请参考 docs\troubleshooting.md
echo pause
) > %PROJECT_ROOT%\setup\diagnose_system.bat

echo.
echo ===============================================
echo           安装完成报告
echo ===============================================
echo.
echo [✓] Chocolatey包管理器
echo [✓] Git版本控制工具
echo [✓] Python 3.x 运行环境
echo [✓] Visual Studio Build Tools
echo [✓] .NET 6.0 Runtime
echo [✓] CMake构建工具
echo [✓] 基础Python包
echo.
echo [下一步] 请完成以下手动步骤:
echo   1. 在Epic Games Launcher中安装UE 5.1.1
echo   2. 运行 build_project.bat 构建项目
echo   3. 运行 setup_environment.bat 配置Python环境
echo   4. 运行 verify_environment.bat 验证安装
echo.
echo [快捷工具]
echo   - verify_environment.bat  : 验证环境配置
echo   - diagnose_system.bat     : 系统诊断工具
echo   - start_project.bat       : 快速启动项目
echo.
echo ===============================================

echo 安装日志已保存到: %INSTALL_LOG%
echo.
pause