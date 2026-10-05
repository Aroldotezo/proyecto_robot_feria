@echo off
setlocal

cd /d "%~dp0"

echo ========================================
echo       CONTROL DEL BRAZO ROBOTICO
echo ========================================
echo.
echo [setup] Buscando Python...

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
echo [ERROR] Python no esta instalado.
echo Instala Python 3.11 o una version posterior.
echo.
pause
exit /b 1

:python_found

echo [setup] Python encontrado.
echo.

if not exist ".venv\Scripts\python.exe" (
    echo [setup] Creando entorno virtual...
    %PYTHON% -m venv .venv

    if errorlevel 1 (
        echo.
        echo [ERROR] No se pudo crear el entorno virtual.
        echo.
        pause
        exit /b 1
    )

    echo [setup] Entorno virtual creado.
    echo.
)

echo [setup] Comprobando dependencias...
echo [setup] Esto puede tardar unos minutos la primera vez.
echo.

".venv\Scripts\python.exe" -m pip install --disable-pip-version-check -r requirements.txt

if errorlevel 1 (
    echo.
    echo [ERROR] Fallo la instalacion de dependencias.
    echo.
    pause
    exit /b 1
)

echo.
echo [setup] Dependencias listas.
echo [setup] Iniciando aplicacion...
echo.

".venv\Scripts\python.exe" -m robotic_arm %*

if errorlevel 1 (
    echo.
    echo [ERROR] La aplicacion termino con un error.
    echo.
    pause
)

endlocal