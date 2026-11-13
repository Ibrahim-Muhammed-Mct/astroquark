@echo off
setlocal
title AstroQuark - Flat Ground

wsl.exe --distribution Ubuntu-24.04 --cd "%~dp0simulation" --exec bash -lc "export GZ_SIM_RESOURCE_PATH=$PWD${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}; exec gz sim -v 3 -r astroquark_flat_world.sdf"
set "rover_status=%errorlevel%"
if not "%rover_status%"=="0" (
    echo Gazebo could not start. Check the error above.
    pause
)
exit /b %rover_status%
