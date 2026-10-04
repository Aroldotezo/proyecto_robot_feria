@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo [setup] Creating virtual environment...
    py -3 -m venv .venv || python -m venv .venv
    if errorlevel 1 (
        echo Could not create the virtual environment. Is Python 3.11 or 3.12 installed?
        pause
        exit /b 1
    )
)

echo [setup] Checking dependencies (the first run takes a few minutes)...
".venv\Scripts\python.exe" -m pip install --quiet --disable-pip-version-check -r requirements.txt
if errorlevel 1 (
    echo Dependency installation failed.
    pause
    exit /b 1
)

".venv\Scripts\python.exe" -m robotic_arm %*
if errorlevel 1 pause
endlocal
