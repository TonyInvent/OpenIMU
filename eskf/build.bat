@echo off
rem Build the ESKF demo with the portable MinGW toolchain in .toolchain\.
rem No admin rights, no MSVC install. Run from anywhere:  eskf\build.bat
setlocal
set "ROOT=%~dp0.."
set "TEMP=%ROOT%\.toolchain\tmp"
set "TMP=%TEMP%"
if not exist "%TEMP%" mkdir "%TEMP%"

set "GPP=%ROOT%\.toolchain\w64devkit\w64devkit\bin\g++.exe"
set "EIGEN=%ROOT%\.toolchain\eigen"

if not exist "%GPP%" (
    echo g++ not found: "%GPP%"
    echo See eskf\README.md "Local toolchain" for how to install w64devkit + Eigen.
    exit /b 1
)
if not exist "%EIGEN%\Eigen\Dense" (
    echo Eigen not found: "%EIGEN%"
    echo   git clone --depth 1 https://gitlab.com/libeigen/eigen.git .toolchain\eigen
    exit /b 1
)

"%GPP%" -O2 -std=c++17 -Wall -Wextra -I"%EIGEN%" -I"%ROOT%\eskf" "%ROOT%\eskf\main.cpp" -o "%ROOT%\eskf\eskf_demo.exe"
if errorlevel 1 (
    echo build FAILED
    exit /b 1
)
echo built: "%ROOT%\eskf\eskf_demo.exe"
endlocal
