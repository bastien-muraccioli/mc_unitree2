#pragma once

#include <chrono>
#include <string>
#include <thread>
#include <Eigen/Core>
#include <vector>
#include <map>

#include <lib/fort.hpp>

//#define __ENABLE_RT_PREEMPT__

#if defined(__ENABLE_RT_PREEMPT__)
#include <pthread.h>
#include "rtapi.h"
#endif

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

// located in unitree_sdk2/example/h1/low_level
#include <h1/low_level/base_state.h>
#include <h1/low_level/data_buffer.hpp>
#include <h1/low_level/motors.hpp>

#include <mc_control/mc_controller.h>
#include "MCControlUnitree2.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using Vector20 = Eigen::Matrix<float, 20, 1>;

namespace mc_unitree
{
  const std::string ROBOT_NAME = "h1";
  
  const int jointIdsToMotorIds[20] =
    {JointIndex::kLeftHipYaw,
     JointIndex::kLeftHipRoll,
     JointIndex::kLeftHipPitch,
     JointIndex::kLeftKnee,
     JointIndex::kLeftAnkle,
     JointIndex::kRightHipYaw,
     JointIndex::kRightHipRoll,
     JointIndex::kRightHipPitch,
     JointIndex::kRightKnee,
     JointIndex::kRightAnkle,
     JointIndex::kWaistYaw,
     JointIndex::kLeftShoulderPitch,
     JointIndex::kLeftShoulderRoll,
     JointIndex::kLeftShoulderYaw,
     JointIndex::kLeftElbow,
     JointIndex::kRightShoulderPitch,
     JointIndex::kRightShoulderRoll,
     JointIndex::kRightShoulderYaw,
     JointIndex::kRightElbow,
     JointIndex::kNotUsedJoint
    };
  
/**
 * @brief Configuration file parameters for mc_unitree
 */
struct H1ConfigParameter
{
  H1ConfigParameter()
  : network_(""), mode_(ControlMode::Position)
  {}
  /* Communication information with a real robot */
  /* Connection network */
  std::string network_;
  /* ControlMode : Position/Velocity/Torque (Velocity is not supported)*/
  ControlMode mode_ = ControlMode::Position;
  
  // Default configuration
  Vector20 q_init_{
    0.0, 0.0, -0.2,  0.6, -0.4,
    0.0, 0.0, -0.2,  0.6, -0.4, // Legs
    0.0, 0.4,  0.0,  0.0, -0.4,
    0.4, 0.0,  0.0, -0.4,       // Torso and arms
    0.0};                       // Unused joint

  // Limits defined by Unitree
  Vector20 q_lim_lower_{
    -0.35, -0.3, -2.0, -0.05, -0.8,
    -0.35, -0.3, -2.0, -0.05, -0.8, // Legs
    -1.65, -2.7, -0.2, -0.8,  -0.85,
    -2.7, -2.8, -3.8, -0.85, // Torso and arms
    0.0};                       // Unused joint
  Vector20 q_lim_upper_{
    0.35, 0.3, 0.9, 1.7, 0.45,
    0.35, 0.3, 0.9, 1.7, 0.45, // Legs
    1.65, 2.7, 2.8, 3.8, 2.4,
    2.7, 0.2, 0.8,  2.4, // Torso and arms
    0.0};                   // Unused joint
  Vector20 qdot_lim_{
    23.0, 23.0, 23.0, 14.0, 9.0,
    23.0, 23.0, 23.0, 14.0, 9.0, // Legs
    23.0, 9.0, 9.0, 9.0, 20.0,
    9.0, 9.0, 9.0, 20.0, // Torso and arms
    0.0};                   // Unused joint
  
  Vector20 kp_{
    1500.0, 1500.0, 1500.0, 1500.0, 1500.0,
         1500.0, 1500.0, 1500.0, 1500.0, 1500.0,
         200.0, 200.0, 100.0, 100.0, 200.0,
         200.0, 100.0, 100.0, 200.0, 0.0};
  
  Vector20 kd_{
    25.0, 25.0, 25.0, 25.0, 5.0,
         25.0, 25.0, 25.0, 25.0, 5.0,
          6.0,  2.0,  2.0,  2.0, 2.0,
          2.0,  2.0,  2.0,  2.0, 0.0};

  Vector20 kp_torque_{
    200.0, 200.0, 200.0, 300.0, 40.0,
         200.0, 200.0, 200.0, 300.0, 40.0,
         50.0, 
         50.0, 50.0, 50.0, 50.0,
         50.0, 50.0, 50.0, 50.0, 
         0.0};
  
  Vector20 kd_torque_{
    5.0, 5.0, 5.0, 6.0, 2.0,
         5.0, 5.0, 5.0, 6.0, 2.0,
          3.0,  
          2.0,  2.0,  2.0, 2.0,
          2.0,  2.0,  2.0,  2.0, 
          0.0};
  
  Vector20 kp_stand_{
    1500.0, 1500.0, 1500.0, 1500.0, 1500.0,
    1500.0, 1500.0, 1500.0, 1500.0, 1500.0, // Legs
    200.0,  200.0,  100.0,  100.0,  200.0,
    200.0,  100.0,  100.0,  200.0, // Torso and arms
    0.0};                          // Unused joint
  
  Vector20 kd_stand_{
    25.0, 25.0, 25.0, 25.0, 25.0,
    25.0, 25.0, 25.0, 25.0, 25.0, // Legs
    6.0,  2.0,  2.0,  2.0,  2.0,
    2.0,  2.0,  2.0,  2.0, // Torso and arms
    0.0};                  // Unused joint
  
  Vector20 tau_ff_{
    0.0,  0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  0.0,
    0.0};
};
  
/**
 * @brief Current sensor values information of H1 robot
 */
struct H1SensorInfo
{
  /* Position(Angle) values */
  std::vector<double> qIn_;
  /* Velocity values */
  std::vector<double> dqIn_;
  /* Torque values */
  std::vector<double> tauIn_;
  /* Orientation sensor */
  Eigen::Vector3d rpyIn_;
  /* Quaternion */
  Eigen::Quaterniond quatIn_;
  /* Accelerometer */
  Eigen::Vector3d accIn_;
  /* Angular velocity */
  Eigen::Vector3d rateIn_;
};

/**
 * @brief Command data for sending to H1 robot
 */
struct H1CommandData
{
  /* Position(Angle) values */
  std::vector<double> qOut_;
  /* Velocity values */
  std::vector<double> dqOut_;
  /* Torque values */
  std::vector<double> tauOut_;
  /* P gains */
  std::vector<double> kpOut_;
  /* D gains */
  std::vector<double> kdOut_;
  /* P gains for Torque Control */
  std::vector<double> kpOutTorque_;
  /* D gains for Torque Control */
  std::vector<double> kdOutTorque_;
};

class H1Control;
template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
class MCControlUnitree2;
  
/**
 * @brief mc_rtc control interface for H1 robot
 */
class H1Control
{
protected:
  typedef enum {
    STATUS_INIT = 0,
    STATUS_WAITING_AIR,
    STATUS_WAITING_GRD,
    STATUS_GAIN_TRANSITION,
    STATUS_RUN,
    STATUS_DAMPING
  } control_status_t;
  
  float control_dt_;
  
  control_status_t status_ = STATUS_INIT;
  control_status_t prev_status_ = STATUS_RUN;

  ControlMode mode_ = ControlMode::Position;
  
  MCControlUnitree2<H1Control, H1SensorInfo, H1CommandData, H1ConfigParameter> * mc_controller_ = nullptr;
  
  mc_rbdyn::Robot* robot_ = nullptr;
  
private:
  /*publisher*/
  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
  /*subscriber*/
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
  
  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<BaseState> base_state_buffer_;
  
  /*! Map from joint order to mc_rtc jointId, because mc_rtc jointId is not defined from 0 */
  std::vector<int> refJointOrderToMCJointId_;
  std::unordered_map<int, int> mcJointIdToJointId_;
  
  Vector20 q_init_;
  Vector20 q_lim_lower_;
  Vector20 q_lim_upper_;
  Vector20 q_dot_lim_lower_;
  Vector20 q_dot_lim_upper_;
  Vector20 kp_;  
  Vector20 kd_;
  Vector20 kp_torque_;  
  Vector20 kd_torque_;
  Vector20 kp_wait_;
  Vector20 kd_wait_;
  Vector20 tau_ff_;
  std::array<float, kNumMotors> tau_des_ = {};

  std::vector<std::string> joint_names_ = {
      "left_hip_yaw_joint",        "left_hip_roll_joint",      "left_hip_pitch_joint", "left_knee_joint",
      "left_ankle_joint",          "right_hip_yaw_joint",      "right_hip_roll_joint", "right_hip_pitch_joint",
      "right_knee_joint",          "right_ankle_joint",        "torso_joint",          "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",  "left_shoulder_yaw_joint",  "left_elbow_joint",     "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "NULL"};
  
  float time_ = 0.f;
  float time_run_ = 0.f;
  const float init_duration_ = 5.f;
  const float interp_duration_ = 0.1f;
  
  float report_dt_ = 0.1f;
  
  // multithreading
  unitree::common::ThreadPtr command_writer_ptr_;
#if defined (__ENABLE_RT_PREEMPT__)
  pthread_t control_thread_;
#else
  unitree::common::ThreadPtr control_thread_ptr_;
#endif
  unitree::common::ThreadPtr report_sensor_ptr_;
  
  /* Control loop status */
  bool running_ = false;
  /* Current sensor values information */
  H1SensorInfo stateIn_;
  
  H1CommandData cmdOut_;
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Refresh the quantities in the tables displayed in the console
  ///
  /// \param[in] init Initialize style and header of tables
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void UpdateTables(bool init = false);
  
  void RecordMotorState(const unitree_go::msg::dds_::LowState_ &msg);
  
  void RecordBaseState(const unitree_go::msg::dds_::LowState_ &msg);
  
  inline bool IsWeakMotor(int motor_index)
  {
    return motor_index == JointIndex::kLeftAnkle ||
      motor_index == JointIndex::kRightAnkle ||
      motor_index == JointIndex::kRightShoulderPitch ||
      motor_index == JointIndex::kRightShoulderRoll ||
      motor_index == JointIndex::kRightShoulderYaw ||
      motor_index == JointIndex::kRightElbow ||
      motor_index == JointIndex::kLeftShoulderPitch ||
      motor_index == JointIndex::kLeftShoulderRoll ||
      motor_index == JointIndex::kLeftShoulderYaw ||
      motor_index == JointIndex::kLeftElbow;
  }
  
public:
  /**
   * @brief Interface constructor and destructor
   * DDS connection with robot using specified parameters.
   * Control level is set to LOW-level.
   *
   * @param config_param Configuration file parameters
   */
  H1Control(MCControlUnitree2<H1Control, H1SensorInfo, H1CommandData, H1ConfigParameter> * mc_controller, mc_rbdyn::Robot * robot, const H1ConfigParameter & config_param);
  
  virtual ~H1Control();
  
  const H1ConfigParameter & config_;
  
  float time() { return time_; }
  
  float timeRun() { return time_run_; }
  
  const H1SensorInfo & getState() const { return stateIn_; }
  
  const H1CommandData & getCommand() const { return cmdOut_; }
  
  const Vector20 & kp() const { return kp_; }
  
  float kp(int i) { return kp_[i]; }

  void setKp(int i, float kp) { kp_[i] = kp; cmdOut_.kpOut_[i] = kp; }

  const Vector20 & kd() const { return kd_; }
  
  float kd(int i) { return kd_[i]; }

  void setKd(int i, float kd) { kd_[i] = kd; cmdOut_.kdOut_[i] = kd; }
  
  const std::vector<int> & refJointOrderToMCJointId() const { return refJointOrderToMCJointId_; }
  
  int refJointOrderToMCJointId(int i) { return refJointOrderToMCJointId_[i]; }
  
  const std::unordered_map<int, int> & mcJointIdToJointId() const { return mcJointIdToJointId_; }
  
  int mcJointIdToJointId(int i) { return mcJointIdToJointId_[i]; }
  
  void setControlMode(const std::string & mode);
  
  void LowCommandWriter();
  
  void LowStateHandler(const void *message);
  
  void Control();
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Basic print of sensor data to the console
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void ReportSensors();
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Launch controller once Enter is pressed
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void endWaiting();
  
  /**
   * @brief Set the initial state values for simulation
   * 
   * @param stance Value defined by RobotModule
   */
  void setInitialState(const std::map<std::string, std::vector<double>> & stance);
  
  /**
   * @brief Loop back the value of "cmdData" to "stateIn"
   * 
   * @param data Command data for sending to H1 robot
   */
  void loopbackState(const H1CommandData & data);
};

} // namespace mc_unitree
