@echo off
setlocal
cd /d "%~dp0"

where py >nul 2>nul
if %errorlevel%==0 (
    set "PYTHON=py -3"
) else (
    set "PYTHON=python"
)

%PYTHON% -c "import cv2, numpy, tkinter" >nul 2>nul
if not %errorlevel%==0 (
    echo Installing required PC packages...
    %PYTHON% -m pip install -r "%~dp0requirements.txt"
    if not %errorlevel%==0 goto :error
)

echo Waiting for MaixCAM on the local network...
%PYTHON% "%~dp0pc_visual_tuner.py" --source auto
if not %errorlevel%==0 goto :error
exit /b 0

:error
echo.
echo Unable to start the visual tuner.
pause
exit /b 1
