# qpik

基于二次规划（QP）的逆运动学（IK）求解器库。

## 简介

qpik 是一个C++ 库，用于解决机器人逆运动学问题。它使用二次规划（Quadratic Programming）方法，支持多种任务、限制和约束条件，能够处理复杂的机器人控制场景。

## 依赖

### 依赖

- **CMake** >= 3.22
- **C++20** 编译器（GCC >= 10 或 Clang >= 12）
- **Eigen3** >= 3.3
- **Pinocchio** - 机器人动力学库
- **OSQP** - 二次规划求解器
- **OsqpEigen** - OSQP 的 Eigen 接口

## 安装和使用


使用提供的构建脚本：

```bash
./build_and_install.bash
```


### 在其他项目中使用

安装后，可以在您的 CMake 项目中使用：

```cmake
find_package(qpik REQUIRED)
target_link_libraries(your_target qpik::qpik)
```


## 项目结构

```
qpik/
├── include/qpik/          # 头文件
│   ├── configuration.hpp  # 配置类
│   ├── solve_ik.hpp       # IK 求解器
│   ├── tasks/             # 任务相关
│   ├── limits/            # 限制相关
│   └── constraints/       # 约束相关
├── src/                   # 源文件
├── cmake/                 # CMake 配置文件
├── CMakeLists.txt         # CMake 主文件
├── .clang-format          # 代码格式化配置
├── .clang-tidy            # 代码检查配置
└── LICENSE                # Apache 2.0 许可证
```

## 许可证

本项目采用 [Apache License 2.0](LICENSE) 许可证。

## 贡献

欢迎提交 Issue 和 Pull Request！


