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
    %PYTHON% -m pip install opencv-python numpy
    if not %errorlevel%==0 goto :error
)

echo Waiting for MaixCAM on the local network...
%PYTHON% "%~dp0tools\maixcam_tuner.py" --source auto
if not %errorlevel%==0 goto :error
exit /b 0

:error
echo.
echo Unable to start the MaixCAM tuner.
pause
exit /b 1
