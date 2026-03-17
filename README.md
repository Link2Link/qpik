# qpik

基于二次规划（QP）的逆运动学（IK）求解器库。

## 简介

qpik 是一个 C++ 库，用于解决机器人逆运动学问题。它使用二次规划（Quadratic Programming）方法，支持多种任务、限制和约束条件，能够处理复杂的机器人控制场景。

## 功能特性

### 支持的任务类型（Tasks）
| 任务 | 描述 |
|------|------|
| `FrameTask` | 末端位姿任务（位置 + 姿态） |
| `CSpaceTargetTask` | 配置空间目标任务 |
| `DampingTask` | 关节速度阻尼任务 |
| `SingularAvoidTask` | 奇异性规避任务 |
| `RelativeFrameTask` | 相对末端任务 |
| `JointTraTask` | 关节轨迹任务 |
| `FrameTraTask` | 末端轨迹任务 |
| `UserDefinedTask` | 用户自定义任务 |
| `JointSoftConstrains` | 关节软约束 |

### 支持的限制类型（Limits - 不等式约束）
| 限制 | 描述 |
|------|------|
| `ConfigurationLimit` | 关节位置限制 |
| `VelocityLimit` | 关节速度限制 |
| `AccelerationLimit` | 关节加速度限制 |
| `CollisionAvoidanceLimit` | 碰撞规避限制 |

### 支持的约束类型（Constraints - 等式约束）
| 约束 | 描述 |
|------|------|
| `DOFFreezingTask` | 关节自由度冻结 |

## 依赖

- **CMake** >= 3.22
- **C++20** 编译器（GCC >= 10 或 Clang >= 12）
- **Eigen3** >= 3.3
- **Pinocchio** - 机器人动力学库
- **OSQP** - 二次规划求解器
- **OsqpEigen** - OSQP 的 Eigen 接口

## 安装

```bash
./build.bash
```

## 在其他项目中使用

安装后，可以在您的 CMake 项目中使用：

```cmake
find_package(qpik REQUIRED)
target_link_libraries(your_target qpik::qpik)
```

## 项目结构

```
qpik/
├── include/qpik/              # 头文件
│   ├── configuration.hpp      # 机器人配置管理
│   ├── solve_ik.hpp           # IK 求解器接口
│   ├── tasks/                 # 任务模块
│   ├── limits/                # 限制模块
│   └── constraints/           # 约束模块
├── src/                       # 源文件
├── demo/                      # 示例代码
├── cmake/                     # CMake 配置文件
└── CMakeLists.txt
```

## 使用示例

```cpp
#include <qpik/qpik.hpp>

// 1. 初始化配置
Configuration config;
config.init(urdf_path, q);

// 2. 创建任务
CSpaceTargetTask cspace_task("CSpaceTargetTask", weight);
cspace_task.set_target(target_q);
std::vector<Task*> tasks = {&cspace_task};

// 3. 创建限制
VelocityLimit velocity_limit("VelocityLimit", config, max_vel);
std::vector<Limit*> limits = {&velocity_limit};

// 4. 创建约束
DOFFreezingTask dof_freezing_task("DOFFreezingTask", config, joint_names);
std::vector<Constraint*> constraints = {&dof_freezing_task};

// 5. 构建并求解 QP 问题
auto problem = construct_qp_problem(config, tasks, limits, constraints, dt);
Eigen::VectorXd dq;
solve_qp_problem(problem, dq);

// 6. 更新配置
q = q + dq * dt;
```

## 数学模型

QP 问题标准形式：
```
minimize    (1/2) * x^T H x + c^T x
subject to  lb <= A * x <= ub
```

其中 `x = dq`（关节速度增量），`H` 和 `c` 由各任务加权组合，`A`, `lb`, `ub` 由限制和约束组合。

## 许可证

本项目采用 [Apache License 2.0](LICENSE) 许可证。

## 贡献

欢迎提交 Issue 和 Pull Request！