@echo off
for %%I in (.) do set parent=%%~nI%%~xI
setlocal ENABLEDELAYEDEXPANSION
set buildType=%1
if "%~1" == "" set /p "buildType=Build type: "
if not exist "bin" mkdir bin
set "CMAKE_BUILD_TYPE_VAL="
REM if /i "%buildType%" == "web" (
REM 	set buildType=Web
REM ) else
if /i "%buildType%" == "release" (
	set buildType=Release
) else (
	set buildType=Debug
)
REM if /i "%buildType%" == "Web" (
REM 	if not exist "build.web" mkdir "build.web"
REM 	cd build.web
REM 	emcmake cmake -DCMAKE_BUILD_TYPE=Release -DPLATFORM=Web ..
REM 	emmake ninja
REM 	cd ..
REM 	exit
REM )
if not exist "build" mkdir "build"
cd build
cmake -DCMAKE_BUILD_TYPE="!buildType!" .. -G "Ninja"
echo Building Executable
Ninja
echo !buildType!
if /i "!buildType!" == "Debug" (
	cd ../bin
	start %parent%
)
endlocal
pause
exit
