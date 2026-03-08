@echo off
echo Starting Elephant Detector (Python 3.12 venv)...
cd /d "%~dp0"
call venv\Scripts\activate
python elephant_detector.py
pause
