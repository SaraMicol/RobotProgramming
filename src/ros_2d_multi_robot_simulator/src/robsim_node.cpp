#include <ros/ros.h>
Eigen::Rotation2Df rot(rob.alpha);
robot_pose.linear() = rot.toRotationMatrix();


Vector2f sensor_offset(rob.radius, 0);
Vector2f sensor_global_pos = robot_pose * sensor_offset;


Isometry2f sensor_pose = robot_pose;
sensor_pose.translation() = sensor_global_pos;


float angle_increment = (as.laser_scan->angle_max - as.laser_scan->angle_min) /
as.laser_scan->ranges.size();


for (size_t i = 0; i < as.laser_scan->ranges.size(); ++i) {
float beam_angle = as.laser_scan->angle_min + angle_increment * i;
Vector2f d(cos(beam_angle), sin(beam_angle));
d = sensor_pose.linear() * d;


as.laser_scan->ranges[i] = grid_map->scanRay(
sensor_pose.translation(),
d,
as.laser_scan->range_max
);
}


sensor_msgs::LaserScan scan_msg;
scan_msg.header.stamp = now;
scan_msg.header.frame_id = as.cfg.frame_id;
scan_msg.angle_min = as.laser_scan->angle_min;
scan_msg.angle_max = as.laser_scan->angle_max;
scan_msg.angle_increment = angle_increment;
scan_msg.time_increment = 0.0;
scan_msg.scan_time = dt;
scan_msg.range_min = as.laser_scan->range_min;
scan_msg.range_max = as.laser_scan->range_max;
scan_msg.ranges = as.laser_scan->ranges;


as.pub.publish(scan_msg);
}
publishLaserVisualization(laser_viz_pub, active_sensors_local, robots, now);


// ===== AGGIORNA CINEMATICA usando current_v_lin/current_v_ang ricevuti via /<ns>/cmd_vel =====
for (size_t i = 0; i < robots.size(); ++i) {
updateUnicycleKinematics(robots[i], dt, grid_map);


if (counter % 10 == 0) {
ROS_INFO("Robot %s: pos=(%.2f, %.2f) theta=%.2f vel=(%.2f, %.2f)",
robots[i].id.c_str(), robots[i].x, robots[i].y, robots[i].alpha,
robots[i].current_v_lin, robots[i].current_v_ang);
}
}
counter++;


map_msg.header.stamp = now;
map_pub.publish(map_msg);
publishTF(tf_broadcaster, robots);


// Pubblica pose e odometry
for (size_t i = 0; i < robots.size(); ++i) {
geometry_msgs::PoseStamped p;
p.header.stamp = now;
p.header.frame_id = "map";
p.pose.position.x = robots[i].x;
p.pose.position.y = robots[i].y;
p.pose.position.z = 0.0;
tf2::Quaternion q;
q.setRPY(0, 0, robots[i].alpha);
p.pose.orientation.x = q.x();
p.pose.orientation.y = q.y();
p.pose.orientation.z = q.z();
p.pose.orientation.w = q.w();
pose_pubs[i].publish(p);


nav_msgs::Odometry odom;
odom.header.stamp = now;
odom.header.frame_id = "odom";
odom.child_frame_id = robots[i].frame_id;
odom.pose.pose = p.pose;
odom.twist.twist.linear.x = robots[i].current_v_lin;
odom.twist.twist.angular.z = robots[i].current_v_ang;
odom_pubs[i].publish(odom);
}


ros::spinOnce();
rate.sleep();
}


return 0;
}