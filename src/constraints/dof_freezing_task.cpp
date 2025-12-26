#include "qpik/constraints/dof_freezing_task.hpp"

namespace qpik {
    DOFFreezingTask::DOFFreezingTask(
        std::string name,
        Configuration &config,
        std::vector<std::string> freezing_dofs)
        : Task(name) {
        this->freezing_dofs = freezing_dofs;
        this->dim = this->freezing_dofs.size();
        this->weight = Eigen::VectorXd::Ones(dim);
        this->gain = 1.0;
        this->lm_damping = 1e-4;

        std::vector<int> indices;
        for (const auto &dof : this->freezing_dofs) {
            indices.push_back(config.joint_ids_map[dof]);
        }
        this->indices = indices;
    }

    Eigen::VectorXd DOFFreezingTask::compute_error(Configuration &config, float dt) {
        return Eigen::VectorXd::Zero(this->indices.size());
    }

    Eigen::MatrixXd DOFFreezingTask::compute_jacobian(Configuration &config, float dt) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(this->dim, config.model_.nv);
        for (int i = 0; i < this->indices.size(); i++) {
            J(i, this->indices[i]) = 1.0;
        }
        return J;
    }

} // namespace qpik