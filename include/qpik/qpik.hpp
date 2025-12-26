#pragma once
#ifndef QPIK_HPP
#define QPIK_HPP

#define DEVELOPMENT_DEBUG

#include <iostream>
#include <qpik/utils.hpp>
#include <qpik/configuration.hpp>

#include <qpik/tasks/task.hpp>
#include <qpik/tasks/CSpaceTargetTask.hpp>
#include <qpik/tasks/frame_task.hpp>


#include "qpik/limits/limit.hpp"
#include "qpik/limits/velocity_limit.hpp"
#include "qpik/limits/configuration_limit.hpp"
#include "qpik/limits/collision_avoidance_limit.hpp"

#include "qpik/constraints/dof_freezing_task.hpp"

#include "qpik/solve_ik.hpp"
#endif // QPIK_HPP
