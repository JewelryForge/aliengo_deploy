#ifndef QUADRUPED_DEPLOY_INCLUDE_ALIENGO_HPP_
#define QUADRUPED_DEPLOY_INCLUDE_ALIENGO_HPP_

#include <iostream>
#include <array>
#include <utility>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <memory>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>

#include "tg.hpp"
#include "io.hpp"
#include "math_utils.hpp"
#include "state.hpp"
#include "policy.hpp"
#include "torch/script.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "alienGo_deploy/GamepadCommand.h"
#include "alienGo_deploy/FloatArray.h"
#include "alienGo_deploy/MultiFloatArray.h"

constexpr float ALIENGO_STANCE_HEIGHT = 0.4;
constexpr std::array<float, 12> ALIENGO_STANCE_POSTURE_ARRAY{0., 0.6435, -1.287, 0., 0.6435, -1.287,
                                                             0., 0.6435, -1.287, 0., 0.6435, -1.287};
constexpr std::array<float, 12> ALIENGO_LYING_POSTURE_ARRAY{
    0., 1.343, -2.696, 0., 1.343, -2.696, 0., 1.343, -2.696, 0., 1.343, -2.696,
};
constexpr std::array<float, 12> ALIENGO_STANCE_FOOT_POSITIONS_ARRAY
    {0., 0., -ALIENGO_STANCE_HEIGHT, 0., 0., -ALIENGO_STANCE_HEIGHT,
     0., 0., -ALIENGO_STANCE_HEIGHT, 0., 0., -ALIENGO_STANCE_HEIGHT};
constexpr std::array<float, 3> ALIENGO_LINK_LENGTHS_ARRAY{0.083, 0.25, 0.25};

inline std::size_t time_stamp() {
  return chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now().time_since_epoch()).count();
}

template<std::size_t N, typename ARRAY1, typename ARRAY2>
static void copy(const ARRAY1 &in, ARRAY2 &out) {
  for (int i = 0; i < N; ++i) out[i] = in[i];
}

class AlienGoComm {
 public:
  explicit AlienGoComm(int inner_freq = 500, int outer_freq = 50)
      : inner_freq_(inner_freq), outer_freq_(outer_freq), num_inner_loops_(inner_freq / outer_freq),
        udp_pub_(UNITREE_LEGGED_SDK::LOWLEVEL),
        safe_(UNITREE_LEGGED_SDK::LeggedType::Aliengo) {
    udp_pub_.InitCmdData(low_cmd_msg_);
    low_cmd_msg_.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    clearCommandMessage();
    udp_pub_.SetSend(low_cmd_msg_);
    udp_pub_.Send();
  }

  ~AlienGoComm() { stopControlThread(); }

  void initComm() {
    active_ = false;
    startControlThread();
    std::this_thread::sleep_for(chrono::milliseconds(100));
    while (true) {
      low_msg_mutex_.lock();
      bool connected = low_state_msg_.tick != 0;
      low_msg_mutex_.unlock();
      if (connected) break;
      std::cout << "NOT CONNECTED" << std::endl;
      std::this_thread::sleep_for(chrono::milliseconds(500));
    }
  }

  void startControlThread() {
    if (not control_loop_thread_.joinable()) {
      low_status_ = true;
      // For thread safety, not to read inner_freq_ directly
      control_loop_thread_ = std::thread(&AlienGoComm::controlLoop, this, inner_freq_);
    }
  }

  void stopControlThread() {
    low_status_ = false;
//    if (control_loop_thread_.joinable()) control_loop_thread_.join();
  }

 private:
  void controlLoop(int freq) {
    auto rate = ros::Rate(freq);
    while (low_status_) {
      controlLoopEvent();
      rate.sleep();
    }
  }

  void setCommandMsg(float Kp = 150, float Kd = 4) {
    for (int i = 0; i < 12; ++i) {
      low_cmd_msg_.motorCmd[i].Kp = Kp;
      low_cmd_msg_.motorCmd[i].Kd = Kd;
      low_cmd_msg_.motorCmd[i].dq = 0;
      low_cmd_msg_.motorCmd[i].q = proc_action_[i];

//      low_cmd_msg_.motorCmd[i].Kp = 0;
//      low_cmd_msg_.motorCmd[i].Kd = 0;
//      low_cmd_msg_.motorCmd[i].dq = 0;
//      low_cmd_msg_.motorCmd[i].q = 0;
//      float residue = proc_action_[i] - low_state_msg_.motorState[i].q;
//      low_cmd_msg_.motorCmd[i].tau = residue * Kp - low_state_msg_.motorState[i].dq * Kd;
    }
  }

 protected:
  void check_safety() {
    float r = low_state_msg_.imu.rpy[0], p = low_state_msg_.imu.rpy[1];
    if (r < -PI / 3 or r > PI / 3) {
      active_ = false;
    }
  }

  void getMotorAnglesWithLock(fArrayRef<12> out) {
    low_state_mutex_.lock();
    for (int i = 0; i < 12; ++i) {
      out[i] = low_state_msg_.motorState[i].q;
    }
    low_state_mutex_.unlock();
  }

  virtual void controlLoopEvent() {
    udp_pub_.Recv();
    low_msg_mutex_.lock();
    udp_pub_.GetRecv(low_state_msg_);
    check_safety();

    if (active_) {
      low_state_mutex_.lock();
      if (inner_loop_cnt_ == num_inner_loops_) {
        proc_action_ = step_action_;
//        print("WAIT");
      } else {
        auto error = step_action_ - proc_action_;
        // smoother interpolation
        proc_action_ += error / (num_inner_loops_ - inner_loop_cnt_);
        ++inner_loop_cnt_;
      }
//      auto current_time_stamp = time_stamp();
//      print(current_time_stamp - last_time_stamp_, inner_loop_cnt_);
//      last_time_stamp_ = current_time_stamp;

      setCommandMsg();
      low_history_mutex_.lock();
      low_cmd_history_.push_back(proc_action_);
      low_history_mutex_.unlock();
      low_state_mutex_.unlock();
    } else {
      clearCommandMessage();
      getMotorAnglesWithLock(proc_action_);
    }

    safe_.PositionLimit(low_cmd_msg_);
    safe_.PowerProtect(low_cmd_msg_, low_state_msg_, 7);
    udp_pub_.SetSend(low_cmd_msg_);
    low_msg_mutex_.unlock();
    if (active_) udp_pub_.Send();
  }

  void applyCommand(fArrayConstRef<12> &cmd) {
    low_state_mutex_.lock();
    step_action_ = cmd;
    inner_loop_cnt_ = 0;
    low_state_mutex_.unlock();

    step_cmd_history_.push_back(cmd);
  }

  void clearCommandMessage() {
    // lock outside the function if needed
    for (int i = 0; i < 12; i++) {
      auto &cmd = low_cmd_msg_.motorCmd[i];
      cmd.mode = 0x0A;   // motor switch to servo (PMSM) mode
      cmd.q = UNITREE_LEGGED_SDK::PosStopF;
      cmd.dq = UNITREE_LEGGED_SDK::VelStopF;
      cmd.Kp = cmd.Kd = cmd.tau = 0;
    }
  }

  const int num_inner_loops_, inner_freq_, outer_freq_;
  std::thread control_loop_thread_;
  // AlienGoBase communication relevant
  UNITREE_LEGGED_SDK::Safety safe_;
  UNITREE_LEGGED_SDK::UDP udp_pub_;
  UNITREE_LEGGED_SDK::LowCmd low_cmd_msg_{};
  UNITREE_LEGGED_SDK::LowState low_state_msg_{};
  std::mutex low_msg_mutex_; // for low_cmd_msg_ & low_state_msg_

  Array12 proc_action_{};
  Array12 step_action_{};
  int inner_loop_cnt_ = 0;
  std::mutex low_state_mutex_; // for proc_action & step_action & inner_loop_cnt_

  std::atomic<bool> low_status_{false}, active_{false};

  StaticQueue<Array12, 100> low_cmd_history_;
  std::mutex low_history_mutex_; // for low_cmd_history_
  StaticQueue<Array12, 10> step_cmd_history_; // Not used inside inner loop

  std::size_t last_time_stamp_ = 0;
};

enum LocomotionState { LYING, L2S, S2L, STANDING, S2M, M2S, MOVING };

class AlienGoBase : public AlienGoComm {
 public:
  explicit AlienGoBase(int inner_freq = 500, int outer_freq = 50)
      : AlienGoComm(inner_freq, outer_freq) {
    robot_vel_sub_ = nh_.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 5, &AlienGoBase::velocityUpdate, this);
    cmd_vel_sub = nh_.subscribe<alienGo_deploy::GamepadCommand>("/gamepad", 1, &AlienGoBase::commandUpdate, this);
    data_tester_ = nh_.advertise<alienGo_deploy::MultiFloatArray>("/test_data", 1);
    applyCommand(STANCE_POSTURE);
    startPolicyThread();
  }

  ~AlienGoBase() { stopPolicyThread(); }

  void startPolicyThread() {
    if (not action_loop_thread_.joinable()) {
      high_status_ = true;
      action_loop_thread_ = std::thread(&AlienGoBase::actionLoop, this, outer_freq_);
    }
  }

  void stopPolicyThread() {
    if (action_loop_thread_.joinable()) {
      high_status_ = false;
      action_loop_thread_.join();
    }
  }

  void setCommand(fArrayConstRef<3> &cmd_vel) {
    high_state_mutex_.lock();
    cmd_vel_ = cmd_vel;
    high_state_mutex_.unlock();
  }

  void inverseKinematics(uint leg, Array3 pos, fArrayRef<3> out) {
    // leg: 0 = forward right; 1 = forward left;
    //      2 = rear right;    3 = rear left.
    // pos: relative to correspondent foot on standing
    float l_shoulder = LINK_LENGTHS[0], l_thigh = LINK_LENGTHS[1], l_shank = LINK_LENGTHS[2];
    if (leg % 2 == 0) l_shoulder *= -1;
    const float &dx = pos[0], &dy = pos[1], &dz = pos[2];
    pos[1] += l_shoulder;
    pos += STANCE_FOOT_POSITIONS.segment<3>(leg * 3);
    while (true) {
      float l_stretch = std::sqrt(Eigen::square(pos).sum() - pow2(l_shoulder));
      float a_hip_bias = std::atan2(dy, dz);
      float sum = std::asin(l_shoulder / std::hypot(dy, dz));
      if (not std::isnan(sum)) {
        float a_hip1 = ang_norm(sum - a_hip_bias), a_hip2 = ang_norm(PI - sum - a_hip_bias);
        out[0] = std::abs(a_hip1) < std::abs(a_hip2) ? a_hip1 : a_hip2;
        float a_stretch = -std::asin(dx / l_stretch);
        if (not std::isnan(a_stretch)) {
          float a_shank = std::acos((pow2(l_shank) + pow2(l_thigh) - pow2(l_stretch))
                                        / (2 * l_shank * l_thigh)) - PI;
          if (not std::isnan(a_shank)) {
            out[2] = a_shank;
            float a_thigh = a_stretch - std::asin(l_shank * std::sin(a_shank) / l_stretch);
            out[1] = a_thigh;
            break;
          }
        }
      }
      pos *= 0.95;
    }
  }

  void inverseKinematicsPatch(fArrayConstRef<12> &pos, fArrayRef<12> out) {
    for (int i = 0; i < 4; ++i) {
      inverseKinematics(i, pos.segment<3>(i * 3), out.segment<3>(i * 3));
    }
  }

 private:
  void velocityUpdate(const nav_msgs::Odometry::ConstPtr &odom) {
    // in main thread
    cam_state_mutex_.lock();
    cam_lin_vel_ = {float(odom->twist.twist.linear.x),
                    float(odom->twist.twist.linear.y),
                    float(odom->twist.twist.linear.z)};
    cam_ang_vel_ = {float(odom->twist.twist.angular.x),
                    float(odom->twist.twist.angular.y),
                    float(odom->twist.twist.angular.z)};
    cam_state_mutex_.unlock();
  }

  void commandUpdate(const alienGo_deploy::GamepadCommand::ConstPtr &cmd) {
    if (cmd->LAS and cmd->RAS) {
      active_ = false;
      locomotion_state_ = LYING;
      return;
    }
    if (cmd->LT > 0.99) {
      if (cmd->A) {
        if (locomotion_state_ == LYING) {
          locomotion_state_ = L2S;
        } else if (locomotion_state_ == STANDING) {
          locomotion_state_ = S2L;
        }
      } else if (cmd->B) {
        if (locomotion_state_ == STANDING) {
          locomotion_state_ = S2M;
        } else if (locomotion_state_ == MOVING) {
          locomotion_state_ = M2S;
        }
      } /* else if (cmd->X) {
        startControlThread();
      } */
    }

    float lin_x = -cmd->left_y, lin_y = -cmd->left_x, rot_z = -cmd->right_x;
    high_state_mutex_.lock();
    if (lin_x != 0. or lin_y != 0.) {
      float norm = std::hypot(lin_x, lin_y);
      cmd_vel_[0] = lin_x / norm;
      cmd_vel_[1] = lin_y / norm;
    } else {
      cmd_vel_[0] = cmd_vel_[1] = 0.;
    }
    if (rot_z > 0.2) cmd_vel_[2] = 1.;
    else if (rot_z < -0.2) cmd_vel_[2] = -1.;
    else cmd_vel_[2] = 0.;
    high_state_mutex_.unlock();
  }

 protected:
  virtual void actionLoop(int freq) {
    auto rate = ros::Rate(freq);
    while (high_status_) {
      actionLoopEvent(freq);
      rate.sleep();
    }
  }

  void actionLoopEvent(int freq) {
    switch (locomotion_state_) {
      case LYING: { break; }
      case STANDING: {
        last_action_ = STANCE_POSTURE;
        break;
      }
      case MOVING: {
        policyEvent(last_action_);
        break;
      }
      case L2S: { // from lying to standing
        if (progress_time_steps_ == 0) {
          getMotorAnglesWithLock(last_action_);
          active_ = true;
        }
        int num_steps_s1 = 0.8 * freq, num_steps_s2 = 1.2 * freq;
        if (progress_time_steps_ < num_steps_s1) {
          auto error = LYING_POSTURE - last_action_;
          last_action_ += error / (num_steps_s1 - progress_time_steps_);
          ++progress_time_steps_;
        } else if (progress_time_steps_ < num_steps_s1 + num_steps_s2) {
          int time_step = progress_time_steps_ - num_steps_s1;
          auto error = STANCE_POSTURE - last_action_;
          last_action_ += error / (num_steps_s2 - time_step);
          ++progress_time_steps_;
        } else {
          progress_time_steps_ = 0;
          locomotion_state_ = STANDING;
        }
        break;
      }
      case S2L: { // from standing to lying
        int num_steps = 1. * freq;
        if (progress_time_steps_ < num_steps) {
          auto error = LYING_POSTURE - last_action_;
          last_action_ += error / (num_steps - progress_time_steps_);
          ++progress_time_steps_;
        } else {
          progress_time_steps_ = 0;
          locomotion_state_ = LYING;
          active_ = false;
        }
        break;
      }
      case S2M: { // from standing to moving
        locomotion_state_ = MOVING;
        policyReset();
        break;
      }
      case M2S: { // from moving to standing
        if (progress_time_steps_ == 0) {
          getMotorAnglesWithLock(last_action_);
        }
        int num_steps = 0.3 * freq;
        if (progress_time_steps_ < num_steps) {
          auto error = STANCE_POSTURE - last_action_;
          last_action_ += error / (num_steps - progress_time_steps_);
          ++progress_time_steps_;
        } else {
          progress_time_steps_ = 0;
          locomotion_state_ = STANDING;
        }
        break;
      }
      default: break;
    }
    applyCommand(last_action_);
  }
  virtual void policyReset() {}
  virtual void policyEvent(fArrayRef<12>) {}

  Array12 STANCE_POSTURE{ALIENGO_STANCE_POSTURE_ARRAY.data()};
  Array12 LYING_POSTURE{ALIENGO_LYING_POSTURE_ARRAY.data()};
  Array12 STANCE_FOOT_POSITIONS{ALIENGO_STANCE_FOOT_POSITIONS_ARRAY.data()};
  Array3 LINK_LENGTHS{ALIENGO_LINK_LENGTHS_ARRAY.data()};

  std::atomic<LocomotionState> locomotion_state_{LYING};
  int progress_time_steps_ = 0;
  Array12 last_action_;

  std::thread action_loop_thread_;
  std::atomic<bool> high_status_{false};

  // velocity calculation relevant
  Vector3 cam_lin_vel_ = Vector3::Zero(), cam_ang_vel_ = Vector3::Zero();
  std::mutex cam_state_mutex_; // for cam_lin_vel_ & cam_ang_vel_
  // high velocity command relevant
  Array3 cmd_vel_ = Array3::Zero();
  std::mutex high_state_mutex_; // for cmd_vel_
  // previous observations relevant
  StaticQueue<std::shared_ptr<ProprioInfo>, 100> obs_history_;
  std::mutex obs_history_mutex_; // for obs_history_
  // ros relevant
  ros::NodeHandle nh_;
  ros::Subscriber robot_vel_sub_, cmd_vel_sub;
  ros::Publisher data_tester_;
};

template<int N>
static alienGo_deploy::FloatArray::Ptr makeFloatArray(const fArray<N> &data) {
  alienGo_deploy::FloatArray::Ptr array(new alienGo_deploy::FloatArray);
  for (int i = 0; i < N; ++i) array->data.push_back(data[i]);
  return array;
}

template<std::size_t N>
static alienGo_deploy::FloatArray::Ptr makeFloatArray(const std::array<float, N> &data) {
  alienGo_deploy::FloatArray::Ptr array(new alienGo_deploy::FloatArray);
  for (int i = 0; i < N; ++i) array->data.push_back(data[i]);
  return array;
}

class AlienGo : public AlienGoBase {
 public:
  explicit AlienGo(const std::string &model_path, int inner_freq = 500, int outer_freq = 50)
      : AlienGoBase(inner_freq, outer_freq),
        policy_(model_path, torch::cuda::is_available() ? torch::kCUDA : torch::kCPU),
        tg_(std::make_shared<VerticalTG>(0.12), 2.0, {0, -PI, -PI, 0}) {}

 private:
  void controlLoopEvent() override {
    AlienGoComm::controlLoopEvent();
    auto data = collectProprioInfo();
    alienGo_deploy::MultiFloatArray multi_array;

//    alienGo_deploy::FloatArray array;
//    multi_array.data.push_back(*makeFloatArray(low_state_msg_.imu.rpy));
//    multi_array.data.push_back(*makeFloatArray(low_state_msg_.imu.gyroscope));
//    multi_array.data.push_back(*makeFloatArray(low_state_msg_.imu.accelerometer));
//    multi_array.data.push_back(array);

//    low_msg_mutex_.lock();
//    multi_array.data.push_back(*makeFloatArray(low_state_msg_.imu.rpy));
//    low_msg_mutex_.unlock();
//    multi_array.data.push_back(*makeFloatArray(data->command));
//    multi_array.data.push_back(*makeFloatArray(data->gravity_vector));
//    multi_array.data.push_back(*makeFloatArray(data->base_linear));

//    multi_array.data.push_back(*makeFloatArray(data->base_angular));
//    multi_array.data.push_back(*makeFloatArray(data->joint_pos));
//    multi_array.data.push_back(*makeFloatArray(data->joint_vel));
//    multi_array.data.push_back(*makeFloatArray(data->joint_pos_target));

//    multi_array.data.push_back(*makeFloatArray());
    data_tester_.publish(multi_array);
  }

  void policyReset() override {
    policy_.clearHistory();
  }

  void policyEvent(fArrayRef<12> out) override {
    low_state_mutex_.lock();
    // copy to make sure obs_history_ is not locked during network inference
    while (obs_history_.is_empty());
    auto proprio_info = *obs_history_.back();
    low_state_mutex_.unlock();
    auto realworld_obs = makeRealWorldObs();
//    auto start = chrono::system_clock::now();
    auto action = policy_.get_action(proprio_info, *realworld_obs);
//    auto end = chrono::system_clock::now();
//    print(chrono::duration_cast<chrono::microseconds>(end - start).count());
    Array12 action_array = Eigen::Map<Array12>(action.data_ptr<float>());
    tg_.update(1. / outer_freq_);
    Array12 priori;
    tg_.getPrioriTrajectory(priori);
    inverseKinematicsPatch(action_array + priori, out);

//    auto current_time_stamp = time_stamp();
//    print(current_time_stamp - last_time_stamp_, inner_loop_cnt_);
//    last_time_stamp_ = current_time_stamp;
  }

  std::shared_ptr<ProprioInfo> collectProprioInfo() {
    auto obs = std::make_shared<ProprioInfo>();
    high_state_mutex_.lock();
    obs->command = cmd_vel_;
    high_state_mutex_.unlock();
    getLinearVelocity(obs->base_linear);

    low_msg_mutex_.lock();
    getGravityVector(low_state_msg_.imu.quaternion, obs->gravity_vector);
    copy<3>(low_state_msg_.imu.gyroscope, obs->base_angular);
    for (int i = 0; i < 12; ++i) {
      obs->joint_pos[i] = low_state_msg_.motorState[i].q;
      obs->joint_vel[i] = low_state_msg_.motorState[i].dq;
    }
    low_msg_mutex_.unlock();

    obs->joint_pos_target = step_cmd_history_.back();
    obs->ftg_frequencies = tg_.freq;
    tg_.getSoftPhases(obs->ftg_phases);

    obs_history_mutex_.lock();
    obs_history_.push_back(obs);
    obs_history_mutex_.unlock();
    return obs;
  }

  std::shared_ptr<RealWorldObservation> makeRealWorldObs() {
    std::shared_ptr<RealWorldObservation> obs(new RealWorldObservation);
    int p0_01 = -int(0.01 * inner_freq_) - 1, p0_02 = -int(0.02 * inner_freq_) - 1;

    obs_history_mutex_.lock();
    assert(not obs_history_.is_empty());
    auto proprio_obs = obs_history_[-1];
    reinterpret_cast<ProprioInfo &>(*obs) = *proprio_obs;
    low_state_mutex_.lock();
    obs->joint_prev_pos_err = proc_action_ - proprio_obs->joint_pos;
    low_state_mutex_.unlock();
//    obs_history_mutex_.unlock();

    low_history_mutex_.lock();
    // in simulation, apply command -> step simulation -> get observation
    // in real world, apply command -> low loop period -> get observation
    auto low_cmd_p0_01 = low_cmd_history_.get_padded(p0_01 - 1),
        low_cmd_p0_02 = low_cmd_history_.get_padded(p0_02 - 1);
//    obs_history_mutex_.lock();
    const auto obs_p0_01 = obs_history_.get_padded(p0_01),
        obs_p0_02 = obs_history_.get_padded(p0_02);
    obs->joint_pos_err_his.segment<12>(0) = low_cmd_p0_01 - obs_p0_01->joint_pos;
    obs->joint_pos_err_his.segment<12>(12) = low_cmd_p0_02 - obs_p0_02->joint_pos;
    low_history_mutex_.unlock();
    obs->joint_vel_his.segment<12>(0) = obs_p0_01->joint_vel;
    obs->joint_vel_his.segment<12>(12) = obs_p0_02->joint_vel;
    obs_history_mutex_.unlock();

    obs->joint_prev_pos_target = step_cmd_history_.get_padded(-2);
    obs->base_frequency = {tg_.base_freq};
    return obs;
  }

  void getLinearVelocity(fArrayRef<3> out) {
    cam_state_mutex_.lock();
    KDL::Twist cam_twist({cam_lin_vel_.x(), cam_lin_vel_.y(), cam_lin_vel_.z()},
                         {cam_ang_vel_.x(), cam_ang_vel_.y(), cam_ang_vel_.z()});
    cam_state_mutex_.unlock();
    KDL::Frame cam2base({0.9703, 0., 0.2419, 0., 1., 0., -0.2419, 0., 0.9703},
                        {0.3312, 0.0173, -0.0045});

    auto base_twist = cam2base * cam_twist;
    out[0] = base_twist.vel.x();
    out[1] = base_twist.vel.y();
    out[2] = base_twist.vel.z();
  }

  void getLinearVelocity2(fArrayRef<3> out) {
    // get base linear velocity in BASE frame
    // w for world, b for base and c for camera
    // b_V_b = b_V_c + b_Ω_c x b_R_c · c_P_cb, then
    // b_V_b = b_R_c · c_V_c + (b_R_c · c_Ω_c) x (b_R_c · c_P_cb)
    // b_V_b = b_R_c · c_V_c + (b_R_c · c_Ω_c) x b_P_cb
    // c_V_c, i.e. cam_lin_vel_; c_Ω_c, i.e. cam_ang_vel_

    Eigen::Matrix3f b_R_c;
    b_R_c << 0.9703, 0., 0.2419, 0., 1., 0., -0.2419, 0., 0.9703;
    const Vector3 b_P_cb(-0.3312, -0.0173, 0.0045);
    cam_state_mutex_.lock();
    out = b_R_c * cam_lin_vel_ + (b_R_c * cam_ang_vel_).cross(b_P_cb);
    cam_state_mutex_.unlock();
  }

  void getGravityVector(const std::array<float, 4> &orientation, fArrayRef<3> out) {
    float w = orientation[0], x = orientation[1], y = orientation[2], z = orientation[3];
    out[0] = 2 * x * z + 2 * y * w;
    out[1] = 2 * y * z - 2 * x * w;
    out[2] = 1 - 2 * x * x - 2 * y * y;
  }

  // action relevant
  TgStateMachine tg_;
  Policy policy_;
};

#endif //QUADRUPED_DEPLOY_INCLUDE_ALIENGO_HPP_
