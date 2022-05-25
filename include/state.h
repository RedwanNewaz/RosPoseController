//
// Created by redwan on 5/24/22.
//

#ifndef POSE_CONTROLLER_STATE_H
#define POSE_CONTROLLER_STATE_H
#include <iostream>
#include <utility>
#include <vector>
#include <cassert>
#include <Eigen/Dense>

class state{
public:
    state(double x, double y, double theta)
    {
        state_ << x, y, theta;
    }

    explicit state(Eigen::Vector3d  state): state_(std::move(state))
    {

    }

    Eigen::VectorXd toError(const state& other)
    {
        double dx = other.state_[0] - state_[0];
        double dy = other.state_[1] - state_[1];
        double rho = sqrt(dx * dx + dy * dy);
        double alpha = fmod(atan2(dy, dx) - state_[0] + M_PI, 2 *M_PI) - M_PI;
        double beta = fmod(other.state_[2] - state_[2] - alpha + M_PI, 2 *M_PI) - M_PI;
        Eigen::Vector3d res;
        res << rho, alpha, beta;
        return res;
    }

    double operator[](int i)
    {
        assert(i < 3 && "state is 3D only");
        return state_[i];
    }

    state operator + (const state& other)
    {
        auto res = this->state_ + other.state_;
        return state(res);
    }

    state operator - (const state& other)
    {
        auto res = this->state_ - other.state_;
        return state(res);
    }

    state operator * (const double& other)
    {
        auto res = this->state_ * other;
        return state(res);
    }

    Eigen::MatrixXd operator * (const Eigen::MatrixXd& other)
    {
        return other * state_;
    }

    Eigen::VectorXd toEigen()
    {
        return state_;
    }
private:
    Eigen::Vector3d state_;
};
#endif //POSE_CONTROLLER_STATE_H
