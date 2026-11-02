@echo off
rem Pulls the robot run logs off the Control Hub (USB) into robot-logs\ and updates its index. Extra arguments are passed on (see pull_logs.py).
python "%~dp0pull_logs.py" %*
