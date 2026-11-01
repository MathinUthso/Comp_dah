#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>

namespace {
// Motion is velocity and effort limited: forcing the joint straight to the target
// in one step makes the flipper slam into the ground and throws the whole robot.
constexpr double kMaxVelocity = 1.0;   // rad/s, matches the URDF joint limit
constexpr double kMaxEffort = 60.0;    // Nm, matches the URDF joint limit; enough
                                       // torque to hold the robot up on its flippers
                                       // (loaded at the 0.45 m tip) while climbing
constexpr double kPositionGain = 6.0;  // 1/s
constexpr double kDeadband = 0.01;     // rad

// Tip rollers: driven with the same linear velocity as the wheels so the robot
// pulls itself along (and up steps) on its flippers. Idle rollers stay free.
constexpr double kIdlerRadius = 0.04;  // m, matches idler_radius in the URDF
constexpr double kIdlerEffort = 5.0;   // Nm
constexpr double kDriveTimeout = 0.5;  // s without a command before rollers are freed

double SteadySeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class FlipperController : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    for (const auto &name : {"front_left_flipper_joint", "front_right_flipper_joint",
                             "rear_left_flipper_joint", "rear_right_flipper_joint"}) {
      auto joint = model->GetJoint(name);
      if (joint) {
        joints_[name] = joint;
        targets_[name] = 0.0;
        joint->SetParam("fmax", 0, kMaxEffort);
        fprintf(stderr, "FlipperController: found joint %s\n", name);
        fflush(stderr);
      } else {
        fprintf(stderr, "FlipperController: joint %s NOT found\n", name);
        fflush(stderr);
      }
    }

    const ignition::math::Vector3d forward =
        model->WorldPose().Rot() * ignition::math::Vector3d::UnitX;
    const ignition::math::Vector3d up = ignition::math::Vector3d::UnitZ;
    for (const auto &name : {"front_left_flipper_idler_joint", "front_right_flipper_idler_joint",
                             "rear_left_flipper_idler_joint", "rear_right_flipper_idler_joint"}) {
      auto joint = model->GetJoint(name);
      if (!joint) {
        fprintf(stderr, "FlipperController: idler joint %s NOT found\n", name);
        fflush(stderr);
        continue;
      }
      idlers_[name] = joint;
      // The joint axis points the other way on the rear (and mirrored) flippers;
      // pick the sign so a positive drive command rolls the robot forward.
      idler_sign_[name] = (joint->GlobalAxis(0).Cross(up).Dot(forward) >= 0.0) ? 1.0 : -1.0;
      joint->SetParam("fmax", 0, 0.0);  // free spinning until a drive command arrives
    }

    sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/flipper_cmd", rclcpp::QoS(10),
        [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
          if (msg->points.empty()) {
            return;
          }
          for (size_t i = 0; i < msg->joint_names.size() &&
                            i < msg->points[0].positions.size();
               ++i) {
            auto it = targets_.find(msg->joint_names[i]);
            if (it == targets_.end()) {
              continue;
            }
            auto joint = joints_.at(msg->joint_names[i]);
            // Never command beyond the URDF limits; the physics engine does not
            // enforce them for a joint that is being driven directly.
            const double target = std::clamp(msg->points[0].positions[i],
                                             joint->LowerLimit(0), joint->UpperLimit(0));
            it->second = target;
            RCLCPP_INFO(node_->get_logger(), "flipper target %s = %.3f rad",
                        msg->joint_names[i].c_str(), target);
          }
        });

    drive_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_sim", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          drive_linear_x_ = msg->linear.x;
          last_drive_seconds_ = SteadySeconds();
        });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        [this](const gazebo::common::UpdateInfo &) {
          for (auto &kv : joints_) {
            auto tit = targets_.find(kv.first);
            if (tit == targets_.end()) {
              continue;
            }
            const double error = tit->second - kv.second->Position(0);
            double velocity = 0.0;
            if (std::abs(error) > kDeadband) {
              velocity = std::clamp(error * kPositionGain, -kMaxVelocity, kMaxVelocity);
            }
            kv.second->SetParam("vel", 0, velocity);
          }

          const bool driving =
              std::abs(drive_linear_x_) > 1e-3 &&
              (SteadySeconds() - last_drive_seconds_) < kDriveTimeout;
          for (auto &kv : idlers_) {
            const double velocity =
                driving ? idler_sign_[kv.first] * drive_linear_x_ / kIdlerRadius : 0.0;
            kv.second->SetParam("fmax", 0, driving ? kIdlerEffort : 0.0);
            kv.second->SetParam("vel", 0, velocity);
          }
        });

    RCLCPP_INFO(node_->get_logger(), "FlipperController loaded with %zu joints and %zu rollers",
                joints_.size(), idlers_.size());
  }

  void Reset() override {
    update_connection_.reset();
  }

private:
  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_sub_;
  std::map<std::string, gazebo::physics::JointPtr> joints_;
  std::map<std::string, double> targets_;
  std::map<std::string, gazebo::physics::JointPtr> idlers_;
  std::map<std::string, double> idler_sign_;
  double drive_linear_x_ = 0.0;
  double last_drive_seconds_ = -1.0;
  gazebo::event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(FlipperController)
