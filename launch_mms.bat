@echo off
if not exist "%~dp0tools\mms\bin\mms.exe" (
    echo Customized MMS is not built. Run: python tools/mms/build_mms.py --setup
    pause
    exit /b 1
)
echo Starting MMRC26 MMS with robot timing and configurable start...
start "" /D "%~dp0" "%~dp0tools\mms\bin\mms.exe"
