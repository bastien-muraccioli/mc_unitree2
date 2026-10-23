#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lib/fort.hpp>

#if defined(__ENABLE_RT_PREEMPT__)
#  include "rtapi.h"
#  include <pthread.h>
#endif

#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// G1 uses the "hg" IDL (H1 and Go2 use the "go2" IDL)
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
// BrainCo Revo2 hands, through unitreerobotics/brainco_hand_service (go2 IDL)
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

#include "MCControlUnitree2.h"
#include <mc_control/mc_controller.h>

namespace mc_unitree
{

const std::string ROBOT_NAME = "g1";

namespace g1
{

constexpr const char * TOPIC_LOWCMD = "rt/lowcmd";
constexpr const char * TOPIC_LOWSTATE = "rt/lowstate";
/* Secondary IMU located in the torso, the main IMU (in LowState) is located in the pelvis */
constexpr const char * TOPIC_IMU_TORSO = "rt/secondary_imu";

/* Number of motors actually used by G1 (29dof version, the 23dof version leaves some unused) */
constexpr int kNumMotors = 29;

/* Size of the motor arrays in the hg LowCmd_/LowState_ messages */
constexpr int kNumMotorsMsg = 35;

/**
 * @brief Motor indices of the G1 (from unitree_sdk2/example/g1/low_level/g1_ankle_swing_example.cpp)
 *
 * For the 23dof version, WaistRoll/WaistPitch and Left/Right WristPitch/WristYaw are not available.
 */
enum JointIndex : int
{
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleRoll = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleRoll = 11,
  WaistYaw = 12,
  WaistRoll = 13,
  WaistPitch = 14,
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,
  LeftWristYaw = 21,
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,
  RightWristYaw = 28
};

/**
 * @brief URDF joint name (mc_g1 / g1_description) to motor index
 */
inline const std::unordered_map<std::string, int> & jointNameToMotorId()
{
  static const std::unordered_map<std::string, int> map = {
      {"left_hip_pitch_joint", LeftHipPitch},
      {"left_hip_roll_joint", LeftHipRoll},
      {"left_hip_yaw_joint", LeftHipYaw},
      {"left_knee_joint", LeftKnee},
      {"left_ankle_pitch_joint", LeftAnklePitch},
      {"left_ankle_roll_joint", LeftAnkleRoll},
      {"right_hip_pitch_joint", RightHipPitch},
      {"right_hip_roll_joint", RightHipRoll},
      {"right_hip_yaw_joint", RightHipYaw},
      {"right_knee_joint", RightKnee},
      {"right_ankle_pitch_joint", RightAnklePitch},
      {"right_ankle_roll_joint", RightAnkleRoll},
      {"waist_yaw_joint", WaistYaw},
      {"waist_roll_joint", WaistRoll},
      {"waist_pitch_joint", WaistPitch},
      {"left_shoulder_pitch_joint", LeftShoulderPitch},
      {"left_shoulder_roll_joint", LeftShoulderRoll},
      {"left_shoulder_yaw_joint", LeftShoulderYaw},
      {"left_elbow_joint", LeftElbow},
      {"left_wrist_roll_joint", LeftWristRoll},
      {"left_wrist_pitch_joint", LeftWristPitch},
      {"left_wrist_yaw_joint", LeftWristYaw},
      {"right_shoulder_pitch_joint", RightShoulderPitch},
      {"right_shoulder_roll_joint", RightShoulderRoll},
      {"right_shoulder_yaw_joint", RightShoulderYaw},
      {"right_elbow_joint", RightElbow},
      {"right_wrist_roll_joint", RightWristRoll},
      {"right_wrist_pitch_joint", RightWristPitch},
      {"right_wrist_yaw_joint", RightWristYaw}};
  return map;
}

/**
 * @brief mode_machine reported by a G1 matching the hardware revision of an mc_g1 robot model
 *
 * See the README of g1_description / unitree_ros: g1_23dof = 1, g1_29dof = 2 (deprecated), g1_23dof_rev_1_0 = 4,
 * g1_29dof_rev_1_0 = 5, g1_29dof_lock_waist_rev_1_0 = 6, g1_<dof>_mode_<N> = N. The _no_hands / _revo2 suffixes
 * (hands) do not change the hardware revision.
 *
 * @return The expected mode_machine, -1 if unknown
 */
inline int expectedModeMachine(std::string name)
{
  for(const std::string suffix : {"_no_hands", "_revo2"})
  {
    if(name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
      name.erase(name.size() - suffix.size());
    }
  }
  static const std::unordered_map<std::string, int> known = {{"g1_23dof", 1},
                                                             {"g1_29dof", 2},
                                                             {"g1_23dof_rev_1_0", 4},
                                                             {"g1_29dof_rev_1_0", 5},
                                                             {"g1_29dof_lock_waist_rev_1_0", 6}};
  auto it = known.find(name);
  if(it != known.end()) { return it->second; }
  auto pos = name.rfind("_mode_");
  if(pos != std::string::npos)
  {
    try
    {
      return std::stoi(name.substr(pos + 6));
    }
    catch(...)
    {
      return -1;
    }
  }
  return -1;
}

/**
 * @brief Ankle (and waist for 29dof) control mode of the G1
 */
enum class AnkleMode : uint8_t
{
  PR = 0, // Series control for Pitch/Roll joints (matches the URDF joints)
  AB = 1  // Parallel control for A/B motors
};

using MotorVector = Eigen::Matrix<float, kNumMotors, 1>;

/* BrainCo Revo2 hands, driven by brainco_hand_service (https://github.com/unitreerobotics/brainco_hand_service):
 * topics rt/brainco/<left|right>/<cmd|state>, 6 motors per hand, positions and speeds normalized in [0, 1]
 * (0: open, 1: closed), motor order [thumb, thumb_aux, index, middle, ring, pinky], mapped to the joints below
 * (same mapping as unitreerobotics/xr_teleoperate). The distal joints are coupled to the proximal ones. */
constexpr int kNumHandMotors = 6;
inline std::array<std::string, kNumHandMotors> revo2MotorJoints(const std::string & side)
{
  return {side + "_thumb_metacarpal_joint", side + "_thumb_proximal_joint", side + "_index_proximal_joint",
          side + "_middle_proximal_joint",  side + "_ring_proximal_joint",  side + "_pinky_proximal_joint"};
}
using HandArray = std::array<float, kNumHandMotors>;

struct MotorCommand
{
  std::array<float, kNumMotors> q_ref = {};
  std::array<float, kNumMotors> dq_ref = {};
  std::array<float, kNumMotors> kp = {};
  std::array<float, kNumMotors> kd = {};
  std::array<float, kNumMotors> tau_ff = {};
};

struct MotorState
{
  std::array<float, kNumMotors> q = {};
  std::array<float, kNumMotors> dq = {};
  std::array<float, kNumMotors> tau = {};
  std::array<int16_t, kNumMotors> temperature = {};
  std::array<uint32_t, kNumMotors> error = {};
};

struct ImuState
{
  /* Quaternion as given by unitree: [w, x, y, z] */
  std::array<float, 4> quat = {1.f, 0.f, 0.f, 0.f};
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
  std::array<float, 3> acc = {};
  int temperature = 0;
};

template<typename T>
class DataBuffer
{
public:
  void SetData(const T & newData)
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData()
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_ ? data_ : nullptr;
  }

  void Clear()
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = nullptr;
  }

private:
  std::shared_ptr<T> data_;
  std::shared_mutex mutex_;
};

inline uint32_t Crc32Core(const uint32_t * ptr, uint32_t len)
{
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for(uint32_t i = 0; i < len; i++)
  {
    xbit = 1u << 31;
    data = ptr[i];
    for(uint32_t bits = 0; bits < 32; bits++)
    {
      if(CRC32 & 0x80000000)
      {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      }
      else
        CRC32 <<= 1;
      if(data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
}

} // namespace g1

/**
 * @brief Configuration file parameters for mc_unitree (G1)
 *
 * All gain vectors are given in MOTOR order (29 values, see g1::JointIndex), for both the 23dof and the 29dof
 * versions, so that the same configuration can be used for both variants and matches unitree_sdk2 examples.
 * Values for motors that do not exist on the robot (e.g. wrist pitch/yaw on 23dof) are ignored.
 */
struct G1ConfigParameter
{
  G1ConfigParameter() : network_("") {}
  /* Communication information with a real robot */
  /* Connection network */
  std::string network_;

  // Default configuration (motor order), only used if the robot module does not provide a stance
  g1::MotorVector q_init_ = (g1::MotorVector() << -0.312, 0.0, 0.0, 0.669, -0.363, 0.0, // Left leg
                             -0.312, 0.0, 0.0, 0.669, -0.363, 0.0, // Right leg
                             0.0, 0.0, 0.0, // Waist
                             0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0, // Left arm
                             0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0) // Right arm
                                .finished();

  // High gains used in Position (and Velocity) control.
  // Values from Unitree xr_teleoperate (G1_29_ArmController), used on the real G1 to hold the whole body:
  // kp_high = 300 / kd_high = 3 for legs and waist, kp_low = 80 / kd_low = 3 for shoulders and elbows,
  // kp_wrist = 40 / kd_wrist = 1.5 for wrists.
  // xr_teleoperate also uses kp_low for the ankle pitch, but it holds the legs of a suspended robot: standing on the
  // ground with joint PD only, the robot is an inverted pendulum on its ankles, which needs an ankle stiffness above
  // m g h_com / 2 ~ 100 Nm/rad per ankle. With 80 the robot tips over (MuJoCo: unitree_mujoco and generated models),
  // with 300 it stands and resists 40 N pushes.
  // These gains (also used while waiting, kp_stand_/kd_stand_) were tested on the real G1 (mode_machine 5, Revo2 hands).
  g1::MotorVector kp_ = (g1::MotorVector() << 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, // Left leg
                         300.0, 300.0, 300.0, 300.0, 300.0, 300.0, // Right leg
                         300.0, 300.0, 300.0, // Waist
                         80.0, 80.0, 80.0, 80.0, 40.0, 40.0, 40.0, // Left arm
                         80.0, 80.0, 80.0, 80.0, 40.0, 40.0, 40.0) // Right arm
                            .finished();

  g1::MotorVector kd_ = (g1::MotorVector() << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, // Left leg
                         3.0, 3.0, 3.0, 3.0, 3.0, 3.0, // Right leg
                         3.0, 3.0, 3.0, // Waist
                         3.0, 3.0, 3.0, 3.0, 1.5, 1.5, 1.5, // Left arm
                         3.0, 3.0, 3.0, 3.0, 1.5, 1.5, 1.5) // Right arm
                            .finished();

  // Gains used during initialization / waiting phases
  g1::MotorVector kp_stand_ = kp_;
  g1::MotorVector kd_stand_ = kd_;

  // Softer gains used to emulate torque control with an inverse PD: q_ref = q + (tau + kd * dq) / kp
  // Legs from unitree_rl_gym (g1 deploy), arms/waist from unitree_sdk2 g1 low-level examples. Must be > 0.
  g1::MotorVector kp_torque_ = (g1::MotorVector() << 100.0, 100.0, 100.0, 150.0, 40.0, 40.0, // Left leg
                                100.0, 100.0, 100.0, 150.0, 40.0, 40.0, // Right leg
                                100.0, 40.0, 40.0, // Waist
                                40.0, 40.0, 40.0, 40.0, 40.0, 20.0, 20.0, // Left arm
                                40.0, 40.0, 40.0, 40.0, 40.0, 20.0, 20.0) // Right arm
                                   .finished();

  g1::MotorVector kd_torque_ = (g1::MotorVector() << 2.0, 2.0, 2.0, 4.0, 2.0, 2.0, // Left leg
                                2.0, 2.0, 2.0, 4.0, 2.0, 2.0, // Right leg
                                2.0, 2.0, 2.0, // Waist
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, // Left arm
                                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0) // Right arm
                                   .finished();

  g1::MotorVector tau_ff_ = g1::MotorVector::Zero();
};

/**
 * @brief Current sensor values information of G1 robot (ref joint order)
 */
struct G1SensorInfo
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
 * @brief Command data for sending to G1 robot (ref joint order)
 */
struct G1CommandData
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
};

class G1Control;
template<typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
class MCControlUnitree2;

/**
 * @brief mc_rtc control interface for G1 robot (23dof and 29dof)
 */
class G1Control
{
protected:
  typedef enum
  {
    STATUS_INIT = 0,
    STATUS_WAITING_AIR,
    STATUS_WAITING_GRD,
    STATUS_GAIN_TRANSITION,
    STATUS_RUN,
    STATUS_DAMPING
  } control_status_t;

  using MCControlG1 = MCControlUnitree2<G1Control, G1SensorInfo, G1CommandData, G1ConfigParameter>;

  MCControlG1 * mc_controller_ = nullptr;

  mc_rbdyn::Robot * robot_ = nullptr;

  float control_dt_;

  std::atomic<control_status_t> status_{STATUS_INIT};
  control_status_t prev_status_ = STATUS_RUN;

private:
  /*publisher*/
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
  /*subscribers*/
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::IMUState_> imutorso_subscriber_;

  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> msc_;

  /* Revo2 hands (left = 0, right = 1) */
  struct Revo2Hand
  {
    std::string side;
    bool present = false;
    /* ref joint order index of each hand motor */
    std::array<int, g1::kNumHandMotors> joint = {};
    /* joint range mapped to [0, 1] */
    g1::HandArray q_min = {};
    g1::HandArray q_max = {};
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::MotorCmds_> publisher;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::MotorStates_> subscriber;
    /* normalized positions */
    g1::DataBuffer<g1::HandArray> state;
    g1::DataBuffer<g1::HandArray> command;
    bool state_warned = false;
  };
  std::array<Revo2Hand, 2> hands_;
  bool hands_enabled_ = false;
  float hand_speed_ = 1.f;
  /*! For each joint of the ref joint order: (hand, motor) if the joint is driven by a Revo2 hand motor */
  std::vector<std::pair<int, int>> handMotorOfJoint_;
  unitree::common::ThreadPtr hand_writer_ptr_;

  void SetupHands(const mc_rtc::Configuration & config);
  void HandCommandWriter();
  void HandStateHandler(size_t hand, const void * message);
  /** Store the hand commands computed by mc_rtc (normalized) */
  void UpdateHandCommands();

  g1::DataBuffer<g1::MotorState> motor_state_buffer_;
  g1::DataBuffer<g1::MotorCommand> motor_command_buffer_;
  g1::DataBuffer<g1::ImuState> imu_pelvis_buffer_;
  g1::DataBuffer<g1::ImuState> imu_torso_buffer_;

  /* Machine type reported by the robot, must be echoed back in LowCmd */
  std::atomic<uint8_t> mode_machine_{0};
  /* mode_machine of the hardware revision of the robot model (-1 if unknown) */
  int expected_mode_machine_ = -1;
  bool check_mode_machine_ = true;
  std::atomic<bool> mode_machine_mismatch_{false};
  g1::AnkleMode ankle_mode_ = g1::AnkleMode::PR;

  /* true when connected to the real robot (network != lo) */
  bool real_robot_ = false;

  /* Number of joints in the ref joint order (23 or 29) */
  size_t n_joints_ = 0;

  /*! Map from ref joint order to motor index */
  std::vector<int> jointIdsToMotorIds_;
  /*! true for motors that are not in the ref joint order (hold position) */
  std::array<bool, g1::kNumMotors> unusedMotor_;
  /*! Position held by unused motors (latched on the first state received) */
  std::array<float, g1::kNumMotors> q_hold_unused_ = {};
  bool q_hold_unused_init_ = false;
  /*! Last error code reported by each motor */
  std::array<uint32_t, g1::kNumMotors> last_motor_error_ = {};

  /*! Map from joint order to mc_rtc jointId, because mc_rtc jointId is not defined from 0 */
  std::vector<int> refJointOrderToMCJointId_;
  std::unordered_map<int, int> mcJointIdToJointId_;

  /* Everything below is in ref joint order (size n_joints_) */
  Eigen::VectorXf q_init_;
  Eigen::VectorXf q_lim_lower_;
  Eigen::VectorXf q_lim_upper_;
  Eigen::VectorXf q_dot_lim_upper_;
  Eigen::VectorXf q_dot_lim_lower_;
  /* Torque limits from the URDF (effort), used by the sensor dashboard */
  Eigen::VectorXf tau_lim_;
  Eigen::VectorXf kp_;
  Eigen::VectorXf kd_;
  Eigen::VectorXf kp_wait_;
  Eigen::VectorXf kd_wait_;
  Eigen::VectorXf kp_torque_;
  Eigen::VectorXf kd_torque_;
  /* Motor order gains used for unused motors */
  g1::MotorVector kp_wait_motor_;
  g1::MotorVector kd_wait_motor_;

  std::vector<std::string> joint_names_;

  /* Name of the body sensor fed with the torso IMU (rt/secondary_imu), empty if the robot module has none */
  std::string torso_imu_sensor_ = "TorsoAccelerometer";
  std::string floating_base_sensor_ = "FloatingBase";
  bool torso_imu_warned_ = false;

  float time_ = 0.f;
  float time_run_ = 0.f;
  const float init_duration_ = 5.f;
  const float interp_duration_ = 0.1f;

  float report_dt_ = 0.1f;

  /* Live sensor dashboard in the terminal (MCControlG1 -s) */
  static inline bool show_sensors_ = false;
  static inline double show_period_ = 0.5;
  static inline bool show_colors_ = true;
  double show_elapsed_ = 0.0;
  /* Last important event (e.g. why the robot went to damping), kept on the dashboard */
  std::mutex event_mutex_;
  std::string last_event_;
  void SetEvent(const std::string & event)
  {
    std::lock_guard<std::mutex> lock(event_mutex_);
    last_event_ = fmt::format("[t = {:.1f} s] {}", time_, event);
  }
  /** Render the sensor dashboard (thread-safe: only reads the sensor/command buffers) */
  std::string SensorDashboard();

  // multithreading
  unitree::common::ThreadPtr command_writer_ptr_;
#if defined(__ENABLE_RT_PREEMPT__)
  pthread_t control_thread_;
  bool control_thread_started_ = false;
  std::atomic<bool> rt_running_{false};
  static void * RTControlLoop(void * arg);
#else
  unitree::common::ThreadPtr control_thread_ptr_;
#endif
  unitree::common::ThreadPtr report_sensor_ptr_;

  /* Current sensor values information */
  G1SensorInfo stateIn_;

  G1CommandData cmdOut_;

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Refresh the status displayed in the console
  ///
  /// \param[in] init Initialize style and header of tables
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void UpdateTables(bool init = false);

  void RecordMotorState(const unitree_hg::msg::dds_::LowState_ & msg);

  void RecordBaseState(const unitree_hg::msg::dds_::LowState_ & msg);

  /** Release the motion control service (sport mode) running on the robot, required for low-level control */
  void ReleaseMotionMode();

  /** Wait for the first rt/lowstate message (throws with a network diagnosis after 10 s) */
  void WaitForLowState(const std::string & network);

  /** Fill the command for the joints in ref joint order with damping only (no stiffness) */
  void DampingCommand(g1::MotorCommand & cmd, const g1::MotorState & ms);

  /** Fill the command for the motors which are not controlled by mc_rtc */
  void UnusedMotorsCommand(g1::MotorCommand & cmd, bool damping);

  /** Set sensors that are not handled by MCControlUnitree2::run (torso IMU, temperatures) */
  void SetAdditionalSensors(const g1::MotorState & ms);

  /** Get a motor-ordered vector from the configuration (29 values), default otherwise */
  static g1::MotorVector ReadMotorVector(const mc_rtc::Configuration & config,
                                         const std::string & key,
                                         const g1::MotorVector & default_value);

  /** Convert a motor-ordered vector into a ref-joint-ordered vector */
  Eigen::VectorXf ToJointOrder(const g1::MotorVector & v) const;

public:
  /**
   * @brief Interface constructor and destructor
   * DDS connection with robot using specified parameters.
   * Control level is set to LOW-level.
   *
   * @param config_param Configuration file parameters
   */
  G1Control(MCControlG1 * mc_controller, mc_rbdyn::Robot * robot, const G1ConfigParameter & config_param);

  virtual ~G1Control();

  /**
   * @brief Show a live dashboard of the sensors in the terminal instead of the status messages
   *
   * Must be called before the interface is created. Enter still moves to the next phase.
   *
   * @param enable Show the dashboard
   * @param period Refresh period [s]
   */
  static void ShowSensors(bool enable, double period = 0.5, bool colors = true)
  {
    show_sensors_ = enable;
    show_period_ = std::max(period, 0.1);
    show_colors_ = colors;
  }

  const G1ConfigParameter & config_;

  float time() { return time_; }

  float timeRun() { return time_run_; }

  const G1SensorInfo & getState() const { return stateIn_; }

  const G1CommandData & getCommand() const { return cmdOut_; }

  const Eigen::VectorXf & kp() const { return kp_; }

  float kp(int i) { return kp_[i]; }

  void setKp(int i, float kp)
  {
    kp_[i] = kp;
    cmdOut_.kpOut_[i] = kp;
  }

  const Eigen::VectorXf & kd() const { return kd_; }

  float kd(int i) { return kd_[i]; }

  void setKd(int i, float kd)
  {
    kd_[i] = kd;
    cmdOut_.kdOut_[i] = kd;
  }

  const std::vector<int> & refJointOrderToMCJointId() const { return refJointOrderToMCJointId_; }

  int refJointOrderToMCJointId(int i) { return refJointOrderToMCJointId_[i]; }

  const std::unordered_map<int, int> & mcJointIdToJointId() const { return mcJointIdToJointId_; }

  int mcJointIdToJointId(int i) { return mcJointIdToJointId_[i]; }

  void LowCommandWriter();

  void LowStateHandler(const void * message);

  void ImuTorsoHandler(const void * message);

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
   * @param data Command data for sending to G1 robot
   */
  void loopbackState(const G1CommandData & data);
};

} // namespace mc_unitree
