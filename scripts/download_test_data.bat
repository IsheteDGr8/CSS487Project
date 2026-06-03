@echo off
setlocal
cd /d "%~dp0\.."
py -3.10 scripts\download_test_data.py
exit /b %ERRORLEVEL%
