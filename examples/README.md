# TRAC-IK 示例程序

## 编译示例

### 前提条件

1. 已编译 TRAC-IK 库
2. 已安装 KDL 和 Eigen3

### 编译步骤

```bash
cd trac_ik/examples
mkdir build
cd build

# Windows (使用 vcpkg)
cmake .. -DCMAKE_BUILD_TYPE=Release ^
         -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake

# Linux/Mac
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build . --config Release
```

### 运行示例

```bash
# Windows
.\Release\basic_ik_example.exe

# Linux/Mac
./basic_ik_example
```

## 示例列表

### basic_ik_example.cpp

基本 IK 求解示例，展示：
- 如何构建 KDL::Chain
- 如何设置关节限位
- 如何创建 TRAC-IK 求解器
- 如何进行 IK 求解
- 如何验证结果

## 更多示例

详细的使用示例请参考 [API_USAGE_EXAMPLES.md](../API_USAGE_EXAMPLES.md)

