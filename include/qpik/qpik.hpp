#pragma once
#ifndef QPIK_HPP
#define QPIK_HPP

/// \file qpik.hpp
/// \brief QP-based IK solver public API 总入口头文件（对外只需包含此文件）
///
/// 包含常用配置、任务（Task）、约束（Limit/Constraint）、求解接口等全部主要API。

#define DEVELOPMENT_DEBUG

#include <iostream>

// === 通用核心 ===
#include <qpik/configuration.hpp>
#include <qpik/utils.hpp>

// === 任务相关（Task family）===
#include "qpik/tasks/JointSoftConstrains.hpp"
#include "qpik/tasks/JointTraTask.hpp"
#include "qpik/tasks/frame_tra_task.hpp"
#include <qpik/tasks/CSpaceTargetTask.hpp> // 配置空间目标任务（q-space）
#include <qpik/tasks/damping_task.hpp>     // 阻尼任务
#include <qpik/tasks/frame_task.hpp>       // 末端位姿任务
#include <qpik/tasks/relative_frame_task.hpp> // 相对末端任务
#include <qpik/tasks/singular_avoid_task.hpp> // 奇异性规避任务
#include <qpik/tasks/task.hpp>                // 任务基类
#include <qpik/tasks/userDefinedTask.hpp>     // 用户自定义任务

// === 不等式约束（Limit family）===
#include "qpik/limits/acceleration_limit.hpp"        // 加速度限制
#include "qpik/limits/collision_avoidance_limit.hpp" // 碰撞规避
#include "qpik/limits/configuration_limit.hpp"       // 配置限制
#include "qpik/limits/limit.hpp"                     // 限制基类
#include "qpik/limits/velocity_limit.hpp"            // 速度限制

// === 等式约束（Constraint family）===
#include "qpik/constraints/dof_freezing_task.hpp" // 关节冻结约束

// === 求解接口 ===

#include "qpik/solve_ik.hpp"

#endif // QPIK_HPP
