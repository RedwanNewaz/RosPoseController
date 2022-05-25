//
// Created by redwan on 5/24/22.
//
#include <ros/ros.h>
#include <array>
#include "pid.h"
#include "state.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

class RosInterface{
public:
    RosInterface()
    {
        nh_ = new ros::NodeHandle("~");
        auto nodeName = ros::this_node::getName();
        ROS_INFO_STREAM(nodeName << " initiated");
        nh_->getParam("sample_time", dt);
        nh_->getParam("rho_gains", rhoGains);
        nh_->getParam("alpha_gains", alphaGains);
        nh_->getParam("beta_gains", betaGains);

        assert(rhoGains.size() == 3 && "gains need to have kp, kd, ki values");
        assert(alphaGains.size() == 3 && "gains need to have kp, kd, ki values");
        assert(betaGains.size() == 3 && "gains need to have kp, kd, ki values");

        nh_->getParam("v_max", v_max);
        nh_->getParam("v_min", v_min);
        nh_->getParam("w_max", w_max);
        nh_->getParam("w_min", w_min);

        // filter
        nh_->getParam("lowPassCoeff", lowPassCoeff);


        string state_topic, cmd_topic, goal_topic;
        nh_->getParam("state_topic", state_topic);
        nh_->getParam("cmd_topic", cmd_topic);
        nh_->getParam("goal_topic", goal_topic);
        state_sub_ = nh_->subscribe(state_topic, 10, &RosInterface::state_callback, this);
        goal_sub_ = nh_->subscribe(goal_topic, 10, &RosInterface::goal_callback, this);

//        cmd_topic = nh_->getNamespace() + cmd_topic;
        cmd_pub_ = nh_->advertise<geometry_msgs::Twist>(cmd_topic, 10);
        v = w = 0;
//        create timer
        timer_ = nh_->createTimer(ros::Duration(dt), &RosInterface::timerCallback, this);
    }

    virtual ~RosInterface()
    {
        delete nh_;
    }

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.orientation, q);

        ROS_INFO("goal pose received = (%lf, %lf, %lf)", x, y, q.getAngle() );
        setRef(x, y, q.getAngle());
    }


    void state_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        updateState(msg->pose.pose.position.x, msg->pose.pose.position.y, q.getAngle());
    }

    void timerCallback(const ros::TimerEvent& event)
    {
        loop();
        geometry_msgs::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;
        cmd_pub_.publish(cmd);
    }

    virtual void updateState(double x, double y, double theta) = 0;
    virtual void loop() = 0;
    virtual void setRef(double x, double y, double theta) = 0;

private:
    ros::NodeHandle *nh_;
    ros::Timer timer_;
    ros::Subscriber state_sub_, goal_sub_;
    ros::Publisher cmd_pub_;


protected:
    //sampling time
    double dt;
    // constraints
    double v_max, v_min;
    double w_max, w_min;
    // kp, kd, ki
    std::vector<double> rhoGains, alphaGains, betaGains;
    // cmd vel
    double rho, v, w;
    //filter
    double lowPassCoeff;
};



class PoseController: public RosInterface
{
public:
    PoseController()
    {
        this->initialized_ = false;


        rho_ = make_shared<PIDImpl>(dt, v_max, v_min, rhoGains[0], rhoGains[1], rhoGains[2]);
        alpha_ = make_shared<PIDImpl>(dt, w_max, w_min, alphaGains[0], alphaGains[1], alphaGains[2]);
        beta_ = make_shared<PIDImpl>(dt, w_max, w_min, betaGains[0], betaGains[1], betaGains[2]);
    }

    void setRef(double x, double y, double theta) override
    {
        this->ref_ = make_shared<state>(x, y, theta);
    }

    void loop() override
    {
        // wait for the state variable to populated
        if(!initialized_ || ref_ == nullptr) return;
        //compute control

        std::tie(rho, v, w) = compute(*est_);

        ROS_INFO("cmd vel = (%lf, %lf, %lf)", rho, v, w);

        if (rho <= 0.1)
            v = w = 0;
    }

    void updateState(double x, double y, double theta) override
    {
        auto current = state(x, y, theta);
        if(!initialized_)
        {
            est_ = make_shared<state>(current);
            initialized_ = true;
        }
        auto filteredState = *est_ * lowPassCoeff + current * (1 - lowPassCoeff);
        this->est_ = make_shared<state>(filteredState);
    }

    tuple<double, double, double> compute(const state& filteredState)
    {
        assert(ref_ != nullptr && "reference hasn't been set yet");


        auto e = ref_->toError(filteredState);
        double v = rho_->compute(e[0]);
        double w = alpha_->compute(e[1]) - beta_->compute(e[2]);

        // we restrict alpha and beta (angle differences) to the range
        // [-pi, pi] to prevent unstable behavior e.g. difference going
        // from 0 rad to 2*pi rad with slight turn

        v = (e[1] > M_PI_2 || e[1] < - M_PI_2) ? - v : v;


        return make_tuple(e[0], v, w);
    }
private:
    shared_ptr<state> ref_, est_;
    shared_ptr<PIDImpl> rho_, alpha_, beta_;
    bool initialized_;

};


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "poseController");
    PoseController pose;

    ros::spin();


    return 0;
}