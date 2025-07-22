# Raylib Kickstart
This template repository offers a nice starting point for a simple raylib
project with a plug-and-play cmakelistsfile that will automatically detect new
source and header files. After creating a new repository based on this one,
you should go to line 18 in [CMakeLists.txt](CMakeLists.txt) and change the
name of the cmake project from RalyibKickstart.
![image](https://github.com/user-attachments/assets/a21651ab-c1e8-45de-b161-7b9792145a93)<br />
The comment says to match the current folder(which in most cases will be the
name of the repository) but it can really be anything. The reason for doing as
the comment says is to allow the provided [build scripts](#how-to-build) to run
the program after building in debug mode.


## How To Build
First, ensure you have a C and C++ compiler (clang on Windows, clang or gcc on
Linux), as well as cmake and emscripten(for web builds) installed.
You will also need Make installed for Linux, and Ninja build installed for
Windows.

Optionally, you can also install raylib locally, but cmake will
install it for you on a per project basis if you don't.

> [!NOTE]
> If you are using an IDE that uses visual studio project files such as
> VSCode, you can generate those project files by createing a folder called
> 'build' inside the project folder, open a terminal inside of the build
> folder, and run: `cmake .. -G "Visual Studio 17 2022"`

> [!NOTE]
> Web builds are not automated using the provided build script for windows
> or the visual studio project ffile generator and must be done manually by
> invoking emscripten from a terminal.

Then, create a file called either 'build.bat'(windows) or 'build.sh'(Linux) and
paste in the respective code from below.

**build.bat:**
```bat
@echo off
for %%I in (.) do set parent=%%~nI%%~xI
setlocal ENABLEDELAYEDEXPANSION
set buildType=%1
if "%~1" == "" set /p "buildType=Build type: "
if not exist "bin" mkdir bin
set "CMAKE_BUILD_TYPE_VAL="
if /i "%buildType%" == "release" (
	set buildType=Release
) else (
	set buildType=Debug
)
if not exist "build" mkdir "build"
cd build
cmake -DCMAKE_BUILD_TYPE="!buildType!" .. -G "Ninja"
echo Building Executable
Ninja
if /i "!buildType!" == "Debug" (
	cd ../bin
	start %parent%
)
endlocal
exit
```
**build.sh:**
```bash
if [[ "$1" ]]; then
	buildType="$1"
else
	read -p "Build type: " buildType
fi
mkdir -p bin
if [[ "${buildType^}" = "Web" ]]; then
	mkdir -p build.web
	cd build.web
	emcmake cmake -DCMAKE_BUILD_TYPE=Release -DPLATFORM=Web ..
	emmake make
	cd ..
	exit
elif [ "${buildType^}" != "Debug" ] && [ "${buildType^}" != "Release" ]; then
	buildType="Debug"
fi
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE="${buildType^}" .. -G "Unix Makefiles"
make
cd ..
if [[ "${buildType^}" = "Debug" ]]; then
	cd bin
	wezterm start --cwd . --always-new-process --class floating ./$(basename $(dirname $PWD))
fi
exit
```
> [!TIP]
> you can skip inputing the build type by adding it as a command line argument
> I.E: `./build.sh debug`  
> In scriptable editors like Neovim, this can be used to automate building in
> debug mode similar to full IDEs like Visual Studio.

Then run `./build.sh` or `start build.bat` from inside the project's root
directory.
