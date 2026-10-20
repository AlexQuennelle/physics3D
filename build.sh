if [[ "$1" ]]; then
	buildType="$1"
else
	read -p "Build type: " buildType
fi
mkdir -p bin
if [[ "${buildType^}" = "Web" ]]; then
	mkdir -p build.web
	emcmake.py cmake -S . -B ./build.web -DCMAKE_BUILD_TYPE=Release -DPLATFORM=Web
	cmake --build ./build.web
	# cd build.web
	# emmake.py make
	# cd ..
	exit
elif [ "${buildType^}" != "Debug" ] && [ "${buildType^}" != "Release" ]; then
	buildType="Debug"
fi
mkdir -p build
cmake -S . -B ./build -G "Ninja" -DCMAKE_BUILD_TYPE="${buildType^}"
cmake --build ./build
exit
