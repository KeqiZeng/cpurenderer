#Makefile

.PHONY: run release clean

PROJECT_NAME=cpurenderer

run:
	@cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
	@cp ./build/compile_commands.json ./compile_commands.json
	@cmake --build build && echo "\n\033[32m========== output ==========\033[0m\n" && ./build/${PROJECT_NAME} assets/african_head

release:
	@cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
	@cmake --build build

clean:
	@rm -rf build
	@rm -rf *.png
