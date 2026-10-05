@echo off
setlocal

cd /d "%~dp0"

echo [setup] Checking Python...

where py >nul 2>&1
if %errorlevel% equ 0 (
    set "PYTHON=py -3"
    goto python_found
)

where python >nul 2>&1
if %errorlevel% equ 0 (
    set "PYTHON=python"
    goto python_found
)

echo.
echo [ERROR] Python is not installed.
echo Please install Python 3.11 or newer.
echo.
pause
exit /b 1

:python_found

if not exist ".venv\Scripts\python.exe" (
    echo [setup] Creating virtual environment...
    %PYTHON% -m venv .venv

    if errorlevel 1 (
        echo.
        echo [ERROR] Could not create the virtual environment.
        echo.
        pause
        exit /b 1
    )
)

echo [setup] Checking dependencies...

".venv\Scripts\python.exe" -m pip install --quiet --disable-pip-version-check -r requirements.txt

if errorlevel 1 (
    echo.
    echo [ERROR] Dependency installation failed.
    echo.
    pause
    exit /b 1
)

echo [setup] Starting application...

".venv\Scripts\python.exe" -m robotic_arm %*

if errorlevel 1 (
    echo.
    echo [ERROR] Application exited with an error.
    echo.
    pause
)

endlocal