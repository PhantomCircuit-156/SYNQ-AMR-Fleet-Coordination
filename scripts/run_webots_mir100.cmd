@echo off
setlocal
for %%I in ("%~dp0..") do set "SYNQ_ROOT=%%~fI"
if not defined WEBOTS_HOME set "WEBOTS_HOME=%LOCALAPPDATA%\Programs\Webots"
set "PATH=%SYNQ_ROOT%\.venv\Scripts;%PATH%"
"%WEBOTS_HOME%\msys64\mingw64\bin\webots.exe" "%SYNQ_ROOT%\webots\worlds\synq_mir100_r1.wbt"
endlocal
