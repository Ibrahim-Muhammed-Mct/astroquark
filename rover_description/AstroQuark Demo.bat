@echo off
setlocal
title AstroQuark - Lunar Demo
wsl.exe --distribution Ubuntu-24.04 --cd "%~dp0." --exec bash -lc "source /opt/ros/jazzy/setup.bash && exec python3 ../scripts/run_astroquark_demo.py"
set "rover_status=%errorlevel%"
if not "%rover_status%"=="0" (
    echo The lunar demo stopped. Check the message above and the run logs.
    pause
)
exit /b %rover_status%
