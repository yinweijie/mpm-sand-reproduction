下列内容主要参考[官方tutorial](https://docs.conan.io/2/tutorial/consuming_packages/build_simple_cmake_project.html)。

# 前置要求
安装conan：

```bash
pip install conan
```

生成conan profile：

```bash
conan profile detect --force
```

# 模板1
在[ConanCenter](https://conan.io/center)搜索需要的库，添加到`[requires]`中：

```cmake
[requires]
zlib/1.3.1
nlohmann_json/3.12.0
fmt/11.2.0
eigen/3.4.0

[generators]
CMakeDeps
CMakeToolchain
```

例如zlib，官网已经给出了如何添加，以及如何在cmake中引入（注意最好不要用`[layout]`这个参数，这个会改变build生成文件的layout，导致clangd找不到compile_commands.json；如果要使用这个参数，参考**模板2**）：

<!-- 这是一张图片，ocr 内容为：CONANFILE.PY CONANFILE.TXT TXT [REQUIRES] ZLIB/1.3.1 [GENERATORS] CMAKEDEPS CMAKETOOLCHAIN [LAYOUT] CMAKE_LAYOUT USEFUL INFORMATION TO TAKE INTO ACCOUNT TO CONSUME THIS LIBRARY: TARGETSHEADERS THESE ARE THE MAIN DECLARED TARGETS: CMAKE PACKAGE NAME(S):ZLIB CMAKE TARGET NAME(S):ZLIB::ZLIB PKG-CONFIG FILE NAME(S):ZLIB.PC NG THE CMAKE FILE NAME AND THE GLOBAL TARGET: A SIMPLE USE CASE USING TH # ... FIND_PACKAGE(ZLIB REQUIRED) TARGET_LINK LIBRARIES(YOUR TARGET ZLIB::ZLIB) -->
![](https://cdn.nlark.com/yuque/0/2025/png/25517747/1755435338695-cd051ab8-69b4-495e-b850-59e08dd4ccc9.png)

根据官网的说明，在cmake中引入即可，直接添加`find_package`和`target_link_libraries`：

```cmake
cmake_minimum_required(VERSION 3.15)
project(compressor)

find_package(ZLIB REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} 
    ZLIB::ZLIB
    nlohmann_json::nlohmann_json
    fmt::fmt
    Eigen3::Eigen)
```

添加好库以后，直接运行下面脚本即可：

```bash
#!/bin/bash

conan install . --output-folder=build --build=missing
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build

```

```cpp
#include <iostream>

#include <zlib.h>
#include <nlohmann/json.hpp>
#include <fmt/core.h>
#include <Eigen/Dense>

int main(void) {
    // Example usage of zlib
    const char* original = "Hello, zlib!";
    uLong original_length = strlen(original) + 1; // +1 for null terminator
    uLong compressed_length = compressBound(original_length);
    Bytef* compressed = (Bytef*)malloc(compressed_length);

    if (compress(compressed, &compressed_length, (const Bytef*)original, original_length) != Z_OK) {
        fprintf(stderr, "Compression failed\n");
        free(compressed);
        return EXIT_FAILURE;
    }

    fmt::print("Original: {}\n", original);
    fmt::print("Compressed length: {}\n", compressed_length);

    // Example usage of nlohmann_json
    nlohmann::json j;
    j["message"] = "Hello, JSON!";
    fmt::print("JSON: {}\n", j.dump());

    // Example usage of Eigen
    Eigen::Vector3d v(1.0, 2.0, 3.0);
    // fmt::print("Eigen Vector: {}\n", v.transpose());
    std::cout << "Eigen Vector: " << v.transpose() << std::endl;

    free(compressed);
    return EXIT_SUCCESS;
}
```

输出：

```bash
> ./build/compressor 
Original: Hello, zlib!
Compressed length: 21
JSON: {"message":"Hello, JSON!"}
Eigen Vector: 1 2 3
```

# 模板2
修改conanfile.txt文件：

```bash
[requires]
zlib/1.3.1
nlohmann_json/3.12.0
fmt/11.2.0
eigen/3.4.0

[generators]
CMakeDeps
CMakeToolchain

[layout]
cmake_layout
```

增加了一个`[layout]`参数，这会让生成的编译的中间文件layout发生变化，如下：

<!-- 这是一张图片，ocr 内容为：BUILD RELEASE CMAKEFILES GENERATORS CMAKE_INSTALL.CMAKE CMAKECACHE.TXT COMPILE_COMMANDS.JSON COMPRESSOR MAKEFILE -->
![](https://cdn.nlark.com/yuque/0/2025/png/25517747/1755438857999-a325f356-1987-45ae-812f-6b78a3dfccdc.png)

即生成到build下的Release或Debug目录下，而不是直接到build目录下。

修改编译脚本：

```bash
#!/bin/bash

set -e

# 方法一
# conan install . --output-folder=build --build=missing
# cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
# cmake --build build

mode=${1:-release}

if [ "$mode" == "release" ]; then
    echo "Building in release mode..."
    # 方法二，release mode
    conan install . --build=missing
    cmake --list-presets
    cmake --preset=conan-release
    cmake --build --preset=conan-release
    ./build/Release/compressor
    rsync -avh ./build/Release/compile_commands.json ./build/
else
    echo "Building in debug mode..."
    ## > conan config home
    ## > cp ~/.conan2/profiles/default ~/.conan2/profiles/debug
    ## > code ~/.conan2/profiles/debug
    ## 
    ## build_type=Debug
    ## 
    # 方法二，debug mode
    conan install . --build=missing -s="build_type=Debug"
    # 上面这行等价于下面这行(注意需要生成debug profile)
    # conan install . --build=missing --profile=debug
    cmake --preset=conan-debug -DCMAKE_BUILD_TYPE=Debug
    cmake --build --preset=conan-debug 
    ./build/Debug/compressor
    rsync -avh ./build/Debug/compile_commands.json ./build/
fi
```

cmake传入`--preset=conan-release`参数，直接用CMakeUserPresets.json配置，这个文件是自动生成的，内容如下：

<!-- 这是一张图片，ocr 内容为：CMAKEUSERPRESETS.JSON X BUILD.SH TEST.SH CMAKELISTS.TXT CON CMAKEUSERPRESETS.JSON > 1 234567 "VERSION" 4, 'VENDOR' 1 CONAN INCLUDE": "BUILD/RELEASE/GENERATORS/CMAKEPRESETS.JSON" 8 -->
![](https://cdn.nlark.com/yuque/0/2025/png/25517747/1755438546424-b122de0c-a86d-4a3f-9743-f8d87903cefa.png)


