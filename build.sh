if [[ "$1" ]]; then
	buildType="$1"
else
	read -p "Build type: " buildType
fi
mkdir -p bin
if [[ "${buildType^}" = "Web" ]]; then
	mkdir -p build.web
	cd build.web
	emcmake.py cmake -DCMAKE_BUILD_TYPE=Release -DPLATFORM=Web ..
	emmake.py make
	cd ..
	exit
elif [ "${buildType^}" != "Debug" ] && [ "${buildType^}" != "Release" ]; then
	buildType="Debug"
fi
mkdir -p build
if uname -a | grep -q "WSL2"; then
	cmake -S . -B ./build -G "Ninja" -DCONFIG_USE_WAYLAND=OFF -DCMAKE_BUILD_TYPE="${buildType^}"
else
	cmake -S . -B ./build -G "Ninja" -DCMAKE_BUILD_TYPE="${buildType^}"
fi
cmake --build ./build
# make
# cd ..
if [[ "${buildType^}" = "Debug" ]]; then
	cd bin
	wezterm start --cwd . --always-new-process --class floating gdb -ex run ./$(basename $(dirname $PWD))
	# wezterm start --cwd . --always-new-process --class floating ./$(basename $(dirname $PWD))
fi
exit
