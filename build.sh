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
cd build
cmake -DCMAKE_BUILD_TYPE="${buildType^}" .. -G "Unix Makefiles"
make
cd ..
if [[ "${buildType^}" = "Debug" ]]; then
	cd bin
	wezterm start --cwd . --always-new-process --class floating gdb -ex run ./$(basename $(dirname $PWD))
	# wezterm start --cwd . --always-new-process --class floating ./$(basename $(dirname $PWD))
fi
exit
