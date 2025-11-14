#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <string>

class SimpleGoalFollower
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Publisher cmd_pub_;

    geometry_msgs::Point current_goal_;
    bool has_goal_;

    std::string robot_ns_;

    // Posizione simulata interna (per esempio)
    double x_, y_;

public:
    SimpleGoalFollower(std::string robot_ns)
        : has_goal_(false), robot_ns_(robot_ns), x_(0.0), y_(0.0)
    {
        std::string goal_topic = "/" + robot_ns_ + "/move_base_simple/goal";
        std::string cmd_topic = "/" + robot_ns_ + "/cmd_vel";

        goal_sub_ = nh_.subscribe(goal_topic, 1, &SimpleGoalFollower::goalCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic, 1);

        ROS_INFO("[%s] Goal follower attivo, ascolta %s", robot_ns_.c_str(), goal_topic.c_str());
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_goal_ = msg->pose.position;
        has_goal_ = true;
        ROS_INFO("[%s] Nuovo goal ricevuto: (%.2f, %.2f)", robot_ns_.c_str(), current_goal_.x, current_goal_.y);
    }

    void spin()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            ros::spinOnce();

            if (has_goal_)
            {
                double dx = current_goal_.x - x_;
                double dy = current_goal_.y - y_;
                double dist = std::hypot(dx, dy);

                geometry_msgs::Twist cmd;

                if (dist < 0.05)
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    cmd_pub_.publish(cmd);
                    ROS_INFO_THROTTLE(2.0, "[%s] Goal raggiunto.", robot_ns_.c_str());
                    has_goal_ = false;
                }
                else
                {
                    double angle = std::atan2(dy, dx);
                    cmd.linear.x = std::min(0.5, dist);
                    cmd.angular.z = 0.0;
                    cmd_pub_.publish(cmd);

                    // Simulazione posizione attuale (non reale)
                    x_ += 0.05 * std::cos(angle);
                    y_ += 0.05 * std::sin(angle);
                }
            }

            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_follower");

    if (argc < 2)
    {
        ROS_ERROR("Uso: goal_follower <robot_namespace>");
        return 1;
    }

    std::string ns = argv[1];
    SimpleGoalFollower follower(ns);
    follower.spin();
    return 0;
}
