@echo off
setlocal
set "ROBOTPY=%USERPROFILE%\anaconda3\envs\py312\python.exe"
if not exist "%ROBOTPY%" set "ROBOTPY=python"
"%ROBOTPY%" "%~dp0scripts\view_robot.py" %*
if errorlevel 1 pause
