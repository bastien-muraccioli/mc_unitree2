
#include "G1Control.h"

#include <mc_control/mc_global_controller.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unistd.h>
#include <iostream>

#if defined(__ENABLE_RT_PREEMPT__)
#  include <time.h>
#endif

using namespace mc_unitree;

g1::MotorVector G1Control::ReadMotorVector(const mc_rtc::Configuration & config,
                                           const std::string & key,
                                           const g1::MotorVector & default_value)
{
  if(!config.has(key)) { return default_value; }
  Eigen::VectorXd v = config(key);
  if(v.size() != g1::kNumMotors)
  {
    mc_rtc::log::error("[mc_unitree] G1: '{}' must have {} values (motor order), got {}. Using default values.", key,
                       g1::kNumMotors, v.size());
    return default_value;
  }
  return v.cast<float>();
}

Eigen::VectorXf G1Control::ToJointOrder(const g1::MotorVector & v) const
{
  Eigen::VectorXf out(n_joints_);
  for(size_t i = 0; i < n_joints_; ++i) { out(i) = jointIdsToMotorIds_[i] >= 0 ? v(jointIdsToMotorIds_[i]) : 0.f; }
  return out;
}

/**
 * @brief Interface constructor and destructor
 * DDS connection with robot using specified parameters.
 * Control level is set to LOW-level.
 *
 * @param config_param Configuration file parameters
 */
G1Control::G1Control(MCControlG1 * mc_controller, mc_rbdyn::Robot * robot, const G1ConfigParameter & config_param)
: mc_controller_(mc_controller), robot_(robot), control_dt_(mc_controller->controller().timestep()),
  config_(config_param)
{
  n_joints_ = robot_->refJointOrder().size();
  mc_rtc::log::info("[mc_unitree] G1 ({}) Number of joints: {}", robot_->name(), n_joints_);
  mc_rtc::log::info("[mc_unitree] G1 Control dt: {}", control_dt_);

  mc_control::MCGlobalController::GlobalConfiguration gconfig("", nullptr);
  if(!gconfig.config.has("Unitree"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_unitree] Missing Unitree configuration");
  }
  auto unitree_config = gconfig.config("Unitree");
  const std::string config_key = robotConfigKey(unitree_config, robot_->name());
  if(config_key.empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_unitree] Missing Unitree {} (or g1) configuration",
                                                     robot_->name());
  }
  auto g1_config = unitree_config(config_key);

  /* Hardware revision expected by the robot model, compared with the mode_machine reported by the robot */
  expected_mode_machine_ = g1::expectedModeMachine(robot_->name());
  g1_config("check-mode-machine", check_mode_machine_);
  if(expected_mode_machine_ > 0)
  {
    mc_rtc::log::info("[mc_unitree] G1: model {} corresponds to mode_machine {}", robot_->name(),
                      expected_mode_machine_);
  }
  else
  {
    mc_rtc::log::warning("[mc_unitree] G1: unknown hardware revision for model {}, mode_machine is not checked",
                         robot_->name());
  }

  /* Map ref joint order to motors */
  const auto & name2motor = g1::jointNameToMotorId();
  unusedMotor_.fill(true);
  jointIdsToMotorIds_.resize(n_joints_, -1);
  joint_names_ = robot_->refJointOrder();
  std::vector<std::string> no_motor_joints;
  for(size_t i = 0; i < n_joints_; ++i)
  {
    const auto & jname = joint_names_[i];
    auto it = name2motor.find(jname);
    if(it == name2motor.end())
    {
      // e.g. hand joints of G1_Revo2: not driven through rt/lowcmd, their state follows the mc_rtc command
      no_motor_joints.push_back(jname);
      continue;
    }
    jointIdsToMotorIds_[i] = it->second;
    unusedMotor_[it->second] = false;
  }
  if(!no_motor_joints.empty())
  {
    std::string names;
    for(const auto & jn : no_motor_joints) { names += (names.empty() ? "" : ", ") + jn; }
    mc_rtc::log::warning("[mc_unitree] G1: {} joints are not G1 motors and are not controlled by this interface "
                         "(their state follows the mc_rtc command): {}",
                         no_motor_joints.size(), names);
  }

  /* Gains, all given in motor order in the configuration */
  const G1ConfigParameter defaults;
  const g1::MotorVector q_init_motor = ReadMotorVector(g1_config, "q_init", defaults.q_init_);
  kp_ = ToJointOrder(ReadMotorVector(g1_config, "kp", defaults.kp_));
  kd_ = ToJointOrder(ReadMotorVector(g1_config, "kd", defaults.kd_));
  kp_wait_motor_ = ReadMotorVector(g1_config, "kp_wait", defaults.kp_stand_);
  kd_wait_motor_ = ReadMotorVector(g1_config, "kd_wait", defaults.kd_stand_);
  kp_wait_ = ToJointOrder(kp_wait_motor_);
  kd_wait_ = ToJointOrder(kd_wait_motor_);
  kp_torque_ = ToJointOrder(ReadMotorVector(g1_config, "kp_torque", defaults.kp_torque_));
  kd_torque_ = ToJointOrder(ReadMotorVector(g1_config, "kd_torque", defaults.kd_torque_));
  q_init_ = ToJointOrder(q_init_motor);

  bool kp_torque_ok = true;
  for(size_t i = 0; i < n_joints_; ++i)
  {
    if(jointIdsToMotorIds_[i] >= 0 && kp_torque_(i) <= 0.f) { kp_torque_ok = false; }
  }
  if(!kp_torque_ok)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_unitree] G1: kp_torque must be strictly positive (inverse PD torque control)");
  }

  /* IMUs: the main IMU (rt/lowstate) is in the pelvis and feeds the default body sensor of mc_g1,
     the secondary IMU (rt/secondary_imu) is in the torso and feeds the "TorsoAccelerometer" body sensor */
  g1_config("torso-imu-sensor", torso_imu_sensor_);
  if(!torso_imu_sensor_.empty() && !robot_->hasBodySensor(torso_imu_sensor_))
  {
    mc_rtc::log::warning("[mc_unitree] G1: robot has no body sensor named {}, the torso IMU will not be used",
                         torso_imu_sensor_);
    torso_imu_sensor_.clear();
  }
  /* mc_mujoco fills the "FloatingBase" body sensor with the simulator ground truth, which does not exist on the
     real robot (nor in unitree_mujoco): it is filled here with the pelvis IMU (same rotation as the pelvis in the
     URDF, imu_in_pelvis_joint rpy = 0) so that controllers reading FloatingBase get the real orientation/gyro.
     Position and linear velocity are NOT measured and stay at their default values. Set to "" to disable. */
  g1_config("floating-base-sensor", floating_base_sensor_);
  if(!floating_base_sensor_.empty()
     && (!robot_->hasBodySensor(floating_base_sensor_) || floating_base_sensor_ == robot_->bodySensor().name()))
  {
    floating_base_sensor_.clear();
  }
  mc_rtc::log::info("[mc_unitree] G1: pelvis IMU -> body sensor {}{}, torso IMU -> body sensor {}",
                    robot_->bodySensor().name(),
                    floating_base_sensor_.empty() ? "" : " and " + floating_base_sensor_ + " (orientation, gyro, acc)",
                    torso_imu_sensor_.empty() ? "(none)" : torso_imu_sensor_);

  stateIn_.qIn_.resize(n_joints_, 0.0);
  stateIn_.dqIn_.resize(n_joints_, 0.0);
  stateIn_.tauIn_.resize(n_joints_, 0.0);
  stateIn_.rpyIn_.setZero();
  stateIn_.quatIn_.setIdentity();
  stateIn_.accIn_.setZero();
  stateIn_.rateIn_.setZero();

  cmdOut_.qOut_.resize(n_joints_, 0.0);
  cmdOut_.dqOut_.resize(n_joints_, 0.0);
  cmdOut_.tauOut_.resize(n_joints_, 0.0);
  cmdOut_.kpOut_.resize(n_joints_);
  cmdOut_.kdOut_.resize(n_joints_);
  for(size_t i = 0; i < n_joints_; i++)
  {
    cmdOut_.kpOut_[i] = kp_[i];
    cmdOut_.kdOut_[i] = kd_[i];
  }

  /* Joint limits from the robot module (URDF), works for both 23dof and 29dof */
  q_lim_lower_.resize(n_joints_);
  q_lim_upper_.resize(n_joints_);
  q_dot_lim_lower_.resize(n_joints_);
  q_dot_lim_upper_.resize(n_joints_);
  tau_lim_ = Eigen::VectorXf::Zero(n_joints_);

  refJointOrderToMCJointId_.resize(n_joints_, -1);
  for(size_t i = 0; i < n_joints_; i++)
  {
    const std::string & jname = joint_names_[i];
    auto mcJointId = robot_->jointIndexByName(jname);
    if(robot_->mbc().q[mcJointId].empty()) continue;

    refJointOrderToMCJointId_[i] = mcJointId;
    mcJointIdToJointId_[mcJointId] = i;

    q_lim_lower_(i) = robot_->ql()[mcJointId][0];
    q_lim_upper_(i) = robot_->qu()[mcJointId][0];
    q_dot_lim_lower_(i) = robot_->vl()[mcJointId][0];
    q_dot_lim_upper_(i) = robot_->vu()[mcJointId][0];
    tau_lim_(i) = robot_->tu()[mcJointId][0];

    /* Joints without motor: their state is the command, start from the stance */
    if(jointIdsToMotorIds_[i] < 0 && robot_->stance().count(jname)) { cmdOut_.qOut_[i] = robot_->stance().at(jname)[0]; }

    /* Overwrite initial q_init_ if stance is set and not provided by the config file */
    if(robot_->stance().count(jname) && !g1_config.has("q_init")) { q_init_(i) = robot_->stance().at(jname)[0]; }
  }

  SetupHands(g1_config);

  const std::string & network = config_param.network_;

  if(!network.empty())
  {
    real_robot_ = (network != "lo");
    int simulation = real_robot_ ? 0 : 1;
    mc_rtc::log::info("[mc_unitree] G1Control: Using network setting: {} ({})", network,
                      real_robot_ ? "real robot" : "simulation");

    /* Initialize */
    unitree::robot::ChannelFactory::Instance()->Init(simulation, network);
    mc_rtc::log::info("Initialize channel factory.");

    /* Subscribers first (passive): used to check the connection before talking to the robot services */
    lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(g1::TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(std::bind(&G1Control::LowStateHandler, this, std::placeholders::_1), 1);

    imutorso_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::IMUState_>(g1::TOPIC_IMU_TORSO));
    imutorso_subscriber_->InitChannel(std::bind(&G1Control::ImuTorsoHandler, this, std::placeholders::_1), 1);

    if(hands_enabled_)
    {
      for(size_t h = 0; h < hands_.size(); ++h)
      {
        auto & hand = hands_[h];
        if(!hand.present) continue;
        hand.publisher.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>(
            "rt/brainco/" + hand.side + "/cmd"));
        hand.publisher->InitChannel();
        hand.subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>(
            "rt/brainco/" + hand.side + "/state"));
        hand.subscriber->InitChannel([this, h](const void * msg) { HandStateHandler(h, msg); }, 1);
      }
      // brainco_hand_service runs at 100 Hz
      hand_writer_ptr_ = unitree::common::CreateRecurrentThreadEx("hand_writer", UT_CPU_ID_NONE, 10000,
                                                                  &G1Control::HandCommandWriter, this);
      mc_rtc::log::info("[mc_unitree] G1: Revo2 hands controlled through rt/brainco/<side>/cmd (requires "
                        "brainco_hand_service running on the robot)");
    }

    if(real_robot_)
    {
      WaitForLowState(network);
      bool release = true;
      g1_config("release-motion-mode", release);
      if(release) { ReleaseMotionMode(); }
      else
      {
        mc_rtc::log::warning("[mc_unitree] G1: release-motion-mode is false, make sure the robot is in debug mode "
                             "(damping: L2+B, then debug mode: L2+R2 on the remote)");
      }
    }

    lowcmd_publisher_.reset(new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(g1::TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();

    // The command is published at the end of each control cycle (see Control()), not by a separate writer thread:
    // a 500 Hz writer thread added up to 2 ms of delay and aliasing with the control frequency (e.g. 400 Hz)

#if defined(__ENABLE_RT_PREEMPT__)
    rt_running_ = true;
    if(pthread_create(&control_thread_, NULL, &G1Control::RTControlLoop, this) == 0)
    {
      control_thread_started_ = true;
    }
    else
    {
      rt_running_ = false;
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_unitree] G1: failed to create the RT control thread");
    }
#else
    int control_period_us = control_dt_ * 1e6;
    control_thread_ptr_ = unitree::common::CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, control_period_us,
                                                                   &G1Control::Control, this);
#endif

    int report_period_us = report_dt_ * 1e6;
    report_sensor_ptr_ = unitree::common::CreateRecurrentThreadEx("report_sensor", UT_CPU_ID_NONE, report_period_us,
                                                                  &G1Control::UpdateTables, this, false);

    // Initialize tables for console display
    UpdateTables(true);
  }
}

G1Control::~G1Control()
{
#if defined(__ENABLE_RT_PREEMPT__)
  rt_running_ = false;
  if(control_thread_started_) { pthread_join(control_thread_, NULL); }
#endif
}

#if defined(__ENABLE_RT_PREEMPT__)
void * G1Control::RTControlLoop(void * arg)
{
  auto * self = static_cast<G1Control *>(arg);
  const long period_ns = static_cast<long>(self->control_dt_ * 1e9);
  if(set_sched_prio(RT_PRIO_MAX - 2, period_ns / 1000) == -1)
  {
    perror("set_sched_prio failed on MCControlUnitree2 (G1).\n");
    return nullptr;
  }
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  while(self->rt_running_)
  {
    self->Control();
    next.tv_nsec += period_ns;
    while(next.tv_nsec >= 1000000000L)
    {
      next.tv_nsec -= 1000000000L;
      next.tv_sec += 1;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
  }
  return nullptr;
}
#endif

void G1Control::WaitForLowState(const std::string & network)
{
  /* rt/lowstate is a plain DDS topic: if nothing arrives, the PC does not see the robot at all */
  const auto start = std::chrono::steady_clock::now();
  while(!motor_state_buffer_.GetData())
  {
    if(std::chrono::steady_clock::now() - start > std::chrono::seconds(10))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_unitree] G1: no data received on {} after 10 s on network interface '{}'. Check that this is the "
          "interface connected to the robot (ip a), that it has an address in the robot network (192.168.123.x, "
          "e.g. 192.168.123.99/24, not .161/.164 used by the robot) and that the robot is powered on",
          g1::TOPIC_LOWSTATE, network);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  mc_rtc::log::success("[mc_unitree] G1: receiving {} (mode_machine {})", g1::TOPIC_LOWSTATE,
                       unsigned(mode_machine_));
}

void G1Control::ReleaseMotionMode()
{
  /* Same as unitree_sdk2/example/g1/low_level: shut down the motion control-related service.
     The robot state is already received at this point (WaitForLowState), so a failure here is not a network
     issue but the motion switcher service (request/response) not answering. */
  msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
  msc_->SetTimeout(5.0f);
  msc_->Init();

  const int max_trials = 5;
  for(int trial = 1; trial <= max_trials; ++trial)
  {
    std::string form, name;
    int32_t ret = msc_->CheckMode(form, name);
    if(ret != 0)
    {
      // 3104 = UT_ROBOT_ERR_CLIENT_API_TIMEOUT: no answer from the service (often the first call, during discovery)
      mc_rtc::log::warning("[mc_unitree] G1: MotionSwitcher CheckMode failed (error code {}{}), attempt {}/{}", ret,
                           ret == 3104 ? ": service did not answer" : "", trial, max_trials);
    }
    else if(name.empty())
    {
      mc_rtc::log::success("[mc_unitree] G1: no motion control service running, low-level control enabled");
      return;
    }
    else
    {
      mc_rtc::log::info("[mc_unitree] G1: motion control service '{}' ({}) active, releasing it", name, form);
      ret = msc_->ReleaseMode();
      if(ret != 0) { mc_rtc::log::warning("[mc_unitree] G1: ReleaseMode failed (error code {})", ret); }
    }
    if(trial < max_trials) { sleep(trial == 1 ? 1 : 5); }
  }
  mc_rtc::log::error_and_throw<std::runtime_error>(
      "[mc_unitree] G1: the robot state is received but the motion switcher service does not answer, so the "
      "Unitree motion control could not be released. Put the robot in debug mode with the remote (damping: L2+B, "
      "then debug mode: L2+R2), then set 'release-motion-mode: false' in the Unitree/g1 configuration");
}

////
// WAITING BEFORE LAUNCHING CONTROLLER
////

// Wait for Enter key press
static void waiting(G1Control * controller)
{
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1000ms);
  std::cin.get();
  controller->endWaiting();
}

void G1Control::endWaiting()
{
  if(status_ == STATUS_WAITING_AIR)
  {
    status_ = STATUS_WAITING_GRD;
    std::thread wait_thread(waiting, this);
    wait_thread.detach();
  }
  else if(status_ == STATUS_WAITING_GRD)
  {
    time_run_ = -control_dt_;
    status_ = STATUS_GAIN_TRANSITION;
  }
}

void G1Control::LowCommandWriter()
{
  const std::shared_ptr<const g1::MotorCommand> mc_tmp_ptr = motor_command_buffer_.GetData();
  if(!mc_tmp_ptr) { return; }

  unitree_hg::msg::dds_::LowCmd_ dds_low_command{};
  dds_low_command.mode_pr() = static_cast<uint8_t>(ankle_mode_);
  dds_low_command.mode_machine() = mode_machine_;

  for(int i = 0; i < g1::kNumMotors; ++i)
  {
    auto & cmd = dds_low_command.motor_cmd().at(i);
    cmd.mode() = 1; // 1: Enable, 0: Disable
    cmd.tau() = mc_tmp_ptr->tau_ff.at(i);
    cmd.q() = mc_tmp_ptr->q_ref.at(i);
    cmd.dq() = mc_tmp_ptr->dq_ref.at(i);
    cmd.kp() = mc_tmp_ptr->kp.at(i);
    cmd.kd() = mc_tmp_ptr->kd.at(i);
  }
  dds_low_command.crc() =
      g1::Crc32Core((uint32_t *)&dds_low_command, (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
  lowcmd_publisher_->Write(dds_low_command);
}

void G1Control::LowStateHandler(const void * message)
{
  const auto & low_state = *(const unitree_hg::msg::dds_::LowState_ *)message;

  if(real_robot_
     && low_state.crc()
            != g1::Crc32Core((uint32_t *)&low_state, (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1))
  {
    mc_rtc::log::error("[mc_unitree] G1: LowState CRC error, message dropped");
    return;
  }

  if(mode_machine_ != low_state.mode_machine())
  {
    const int mm = low_state.mode_machine();
    if(mode_machine_ == 0) { mc_rtc::log::info("[mc_unitree] G1 type (mode_machine): {}", mm); }
    mode_machine_ = low_state.mode_machine();
    if(real_robot_ && expected_mode_machine_ > 0 && mm != expected_mode_machine_)
    {
      mc_rtc::log::error("[mc_unitree] G1: the robot reports mode_machine {} but the model {} is made for mode_machine "
                         "{}. Use MainRobot: G1_mode_{} (mc_g1).",
                         mm, robot_->name(), expected_mode_machine_, mm);
      if(check_mode_machine_) { mode_machine_mismatch_ = true; }
    }
  }

  RecordMotorState(low_state);
  RecordBaseState(low_state);
}

void G1Control::ImuTorsoHandler(const void * message)
{
  const auto & imu = *(const unitree_hg::msg::dds_::IMUState_ *)message;
  g1::ImuState imu_tmp;
  imu_tmp.quat = imu.quaternion();
  imu_tmp.rpy = imu.rpy();
  imu_tmp.omega = imu.gyroscope();
  imu_tmp.acc = imu.accelerometer();
  imu_tmp.temperature = imu.temperature();
  imu_torso_buffer_.SetData(imu_tmp);
}

void G1Control::RecordMotorState(const unitree_hg::msg::dds_::LowState_ & msg)
{
  g1::MotorState ms_tmp;
  for(int i = 0; i < g1::kNumMotors; ++i)
  {
    const auto & m = msg.motor_state()[i];
    ms_tmp.q.at(i) = m.q();
    ms_tmp.dq.at(i) = m.dq();
    ms_tmp.tau.at(i) = m.tau_est();
    ms_tmp.temperature.at(i) = std::max(m.temperature()[0], m.temperature()[1]);
    ms_tmp.error.at(i) = unusedMotor_[i] ? 0 : m.motorstate();
  }

  motor_state_buffer_.SetData(ms_tmp);
}

void G1Control::RecordBaseState(const unitree_hg::msg::dds_::LowState_ & msg)
{
  g1::ImuState bs_tmp;
  bs_tmp.omega = msg.imu_state().gyroscope();
  bs_tmp.quat = msg.imu_state().quaternion();
  bs_tmp.rpy = msg.imu_state().rpy();
  bs_tmp.acc = msg.imu_state().accelerometer();
  bs_tmp.temperature = msg.imu_state().temperature();

  imu_pelvis_buffer_.SetData(bs_tmp);
}

void G1Control::ReportSensors()
{
  const auto bs_tmp_ptr = imu_pelvis_buffer_.GetData();
  const auto ms_tmp_ptr = motor_state_buffer_.GetData();
  if(bs_tmp_ptr)
  {
    mc_rtc::log::info("Pelvis IMU RPY: [{:.4f}, {:.4f}, {:.4f}]", bs_tmp_ptr->rpy.at(0), bs_tmp_ptr->rpy.at(1),
                      bs_tmp_ptr->rpy.at(2));
    mc_rtc::log::info("Gyro: [{:.4f}, {:.4f}, {:.4f}]", bs_tmp_ptr->omega.at(0), bs_tmp_ptr->omega.at(1),
                      bs_tmp_ptr->omega.at(2));
    mc_rtc::log::info("Accelerometer: [{:.4f}, {:.4f}, {:.4f}]", bs_tmp_ptr->acc.at(0), bs_tmp_ptr->acc.at(1),
                      bs_tmp_ptr->acc.at(2));
  }
  if(ms_tmp_ptr)
  {
    for(size_t i = 0; i < n_joints_; ++i)
    {
      auto motorId = jointIdsToMotorIds_[i];
      if(motorId < 0) continue;
      mc_rtc::log::info("{}: q = {:.4f}, dq = {:.4f}, tau = {:.4f}, temp = {}", joint_names_[i],
                        ms_tmp_ptr->q.at(motorId), ms_tmp_ptr->dq.at(motorId), ms_tmp_ptr->tau.at(motorId),
                        ms_tmp_ptr->temperature.at(motorId));
    }
  }
}

void G1Control::UpdateTables(bool init)
{
  if(show_sensors_)
  {
    show_elapsed_ += init ? show_period_ : report_dt_;
    if(show_elapsed_ + 1e-6 < show_period_) return;
    show_elapsed_ = 0.0;
    // Clear the terminal and redraw the dashboard in place
    std::cout << "\033[2J\033[H" << SensorDashboard() << std::flush;
    return;
  }
  const control_status_t status = status_;
  if(status != prev_status_)
  {
    prev_status_ = status;
    switch(status)
    {
      case STATUS_INIT:
        mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
        mc_rtc::log::info("    ┃      Initialization      ┃");
        mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
        break;
      case STATUS_WAITING_AIR:
        mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
        mc_rtc::log::info("    ┃    Waiting in the air    ┃");
        mc_rtc::log::info("    ┃   (press Enter to go on) ┃");
        mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
        break;
      case STATUS_WAITING_GRD:
        mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
        mc_rtc::log::info("    ┃   Waiting on the ground  ┃");
        mc_rtc::log::info("    ┃   (press Enter to run)   ┃");
        mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
        break;
      case STATUS_GAIN_TRANSITION:
        mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
        mc_rtc::log::info("    ┃   PD Gains Transition    ┃");
        mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
        break;
      case STATUS_RUN:
        mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
        mc_rtc::log::info("    ┃    Running Controller    ┃");
        mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
        break;
      case STATUS_DAMPING:
        mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
        mc_rtc::log::info("    ┃    Emergency Damping!    ┃");
        mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
        break;
    }
  }
}

namespace
{
// ANSI styles for the lines outside of the tables
struct Ansi
{
  bool on;
  std::string operator()(const char * code, const std::string & text) const
  {
    return on ? std::string("\033[") + code + "m" + text + "\033[0m" : text;
  }
};
} // namespace

std::string G1Control::SensorDashboard()
{
  const auto ms = motor_state_buffer_.GetData();
  const auto mc = motor_command_buffer_.GetData();
  const auto pelvis = imu_pelvis_buffer_.GetData();
  const auto torso = imu_torso_buffer_.GetData();
  constexpr double rad2deg = 180.0 / M_PI;
  const Ansi ansi{show_colors_};
  // Colors of the table cells (only when colors are enabled)
  auto paint = [&](fort::char_table & t, size_t row, size_t col, fort::color c, bool bold = false)
  {
    if(!show_colors_) return;
    t[row][col].set_cell_content_fg_color(c);
    if(bold) t[row][col].set_cell_content_text_style(fort::text_style::bold);
  };
  auto header = [&](fort::char_table & t, size_t ncols)
  {
    if(!show_colors_) return;
    for(size_t c = 0; c < ncols; ++c)
    {
      t[0][c].set_cell_content_fg_color(fort::color::light_cyan);
      t[0][c].set_cell_content_text_style(fort::text_style::bold);
    }
  };

  std::string out;
  const control_status_t status = status_;
  const char * status_name = "";
  const char * hint = "";
  const char * status_style = "1";
  switch(status)
  {
    case STATUS_INIT:
      status_name = "INITIALIZATION";
      hint = "moving to the initial posture";
      status_style = "1;36"; // bold cyan
      break;
    case STATUS_WAITING_AIR:
      status_name = "WAITING IN THE AIR";
      hint = "press Enter once the robot is on the ground";
      status_style = "1;33"; // bold yellow
      break;
    case STATUS_WAITING_GRD:
      status_name = "WAITING ON THE GROUND";
      hint = "press Enter to start the controller";
      status_style = "1;33";
      break;
    case STATUS_GAIN_TRANSITION:
      status_name = "PD GAINS TRANSITION";
      status_style = "1;33";
      break;
    case STATUS_RUN:
      status_name = "RUNNING CONTROLLER";
      status_style = "1;32"; // bold green
      break;
    case STATUS_DAMPING:
      status_name = "EMERGENCY DAMPING";
      hint = "the robot is not controlled anymore, stop the program";
      status_style = "1;37;41"; // bold white on red
      break;
  }
  const unsigned mm = mode_machine_;
  const std::string mm_text = fmt::format("mode_machine {} (model: {})", mm, expected_mode_machine_);
  const char * mm_style = mm == 0 ? "33" : (expected_mode_machine_ > 0 && int(mm) != expected_mode_machine_ ? "1;31" : "32");
  out += fmt::format("{} | {} | {} | t = {:.1f} s | mode {}\n", ansi("1", "MCControlG1"), robot_->name(),
                     ansi(mm_style, mm_text), time_,
                     mc_controller_->mode() == ControlMode::Position
                         ? "Position"
                         : (mc_controller_->mode() == ControlMode::Velocity ? "Velocity" : "Torque"));
  out += ansi(status_style, fmt::format(" {} ", status_name)) + (hint[0] ? std::string("  ") + hint : "") + "\n";
  {
    std::lock_guard<std::mutex> lock(event_mutex_);
    if(!last_event_.empty()) { out += ansi("1;31", "!!! " + last_event_) + "\n"; }
  }
  out += "\n";

  /* IMUs */
  {
    fort::char_table t;
    t.set_border_style(FT_BASIC_STYLE);
    t << fort::header << "IMU" << "topic" << "roll [deg]" << "pitch [deg]" << "yaw [deg]" << "gyro [rad/s]"
      << "acc [m/s2]" << "T [C]" << fort::endr;
    header(t, 8);
    size_t row = 1;
    auto add = [&](const char * name, const char * topic, const std::shared_ptr<const g1::ImuState> & imu, bool required)
    {
      if(!imu)
      {
        t << name << topic << "no data" << "" << "" << "" << "" << "" << fort::endr;
        paint(t, row, 2, required ? fort::color::red : fort::color::dark_gray, required);
        ++row;
        return;
      }
      t << name << topic << fmt::format("{:+.2f}", imu->rpy[0] * rad2deg) << fmt::format("{:+.2f}", imu->rpy[1] * rad2deg)
        << fmt::format("{:+.2f}", imu->rpy[2] * rad2deg)
        << fmt::format("{:+.3f} {:+.3f} {:+.3f}", imu->omega[0], imu->omega[1], imu->omega[2])
        << fmt::format("{:+.2f} {:+.2f} {:+.2f}", imu->acc[0], imu->acc[1], imu->acc[2]) << imu->temperature
        << fort::endr;
      // Tilt (roll/pitch): yellow above 10 deg, red above 25 deg
      for(size_t k = 0; k < 2; ++k)
      {
        const double a = std::abs(imu->rpy[k] * rad2deg);
        if(a > 25.) paint(t, row, 2 + k, fort::color::red, true);
        else if(a > 10.) paint(t, row, 2 + k, fort::color::yellow);
      }
      ++row;
    };
    add("pelvis (main)", "rt/lowstate", pelvis, true);
    add("torso", "rt/secondary_imu", torso, false);
    out += t.to_string() + "\n";
  }

  /* Joints */
  if(!ms)
  {
    out += ansi("1;31", "No motor state received on rt/lowstate yet") + "\n";
    return out;
  }
  {
    fort::char_table t;
    t.set_border_style(FT_BASIC_STYLE);
    t << fort::header << "motor" << "joint" << "q [rad]" << "q_ref [rad]" << "dq [rad/s]" << "tau [Nm]" << "kp" << "kd"
      << "T [C]" << "limits" << "error" << fort::endr;
    header(t, 11);
    size_t row = 1;
    for(size_t i = 0; i < n_joints_; ++i)
    {
      const int m = jointIdsToMotorIds_[i];
      if(m < 0) continue; // e.g. Revo2 fingers, see below
      const float q = ms->q[m];
      const float margin = std::min(q - q_lim_lower_(i), q_lim_upper_(i) - q);
      const std::string lim = margin < 0.f ? "OUT" : (margin < 0.1f ? "near" : "");
      t << m << joint_names_[i] << fmt::format("{:+.3f}", q)
        << (mc ? fmt::format("{:+.3f}", mc->q_ref[m]) : std::string("-")) << fmt::format("{:+.3f}", ms->dq[m])
        << fmt::format("{:+.2f}", ms->tau[m]) << (mc ? fmt::format("{:.0f}", mc->kp[m]) : std::string("-"))
        << (mc ? fmt::format("{:.1f}", mc->kd[m]) : std::string("-")) << ms->temperature[m] << lim
        << (ms->error[m] ? std::to_string(ms->error[m]) : std::string("")) << fort::endr;
      // Tracking error |q_ref - q|: yellow above 0.1 rad, red above 0.3 rad
      if(mc)
      {
        const float e = std::abs(mc->q_ref[m] - q);
        if(e > 0.3f) paint(t, row, 2, fort::color::red, true);
        else if(e > 0.1f) paint(t, row, 2, fort::color::yellow);
      }
      // Velocity: yellow above 50% of the URDF limit
      if(q_dot_lim_upper_(i) > 0.f && std::abs(ms->dq[m]) > 0.5f * q_dot_lim_upper_(i)) paint(t, row, 4, fort::color::yellow);
      // Torque: yellow above 60% of the URDF effort limit, red above 90%
      if(tau_lim_(i) > 0.f)
      {
        const float r = std::abs(ms->tau[m]) / tau_lim_(i);
        if(r > 0.9f) paint(t, row, 5, fort::color::red, true);
        else if(r > 0.6f) paint(t, row, 5, fort::color::yellow);
      }
      // Gains: dimmed when the motor is not stiff (damping)
      if(mc && mc->kp[m] == 0.f) paint(t, row, 6, fort::color::dark_gray);
      // Temperature: yellow from 60 C, red from 75 C
      if(ms->temperature[m] >= 75) paint(t, row, 8, fort::color::red, true);
      else if(ms->temperature[m] >= 60) paint(t, row, 8, fort::color::yellow);
      // Limits and motor errors
      if(margin < 0.f) paint(t, row, 9, fort::color::red, true);
      else if(margin < 0.1f) paint(t, row, 9, fort::color::yellow);
      if(ms->error[m]) paint(t, row, 10, fort::color::red, true);
      ++row;
    }
    t.column(0).set_cell_text_align(fort::text_align::right);
    for(size_t c = 2; c < 9; ++c) { t.column(c).set_cell_text_align(fort::text_align::right); }
    out += t.to_string();
    size_t held = 0;
    for(int m = 0; m < g1::kNumMotors; ++m) { held += unusedMotor_[m] ? 1 : 0; }
    if(held) { out += ansi("2", fmt::format("{} motor(s) not in the model held at their initial position", held)) + "\n"; }
    out += "\n";
  }

  /* Revo2 hands */
  if(hands_enabled_)
  {
    fort::char_table t;
    t.set_border_style(FT_BASIC_STYLE);
    t << fort::header << "hand" << "" << "thumb" << "thumb_aux" << "index" << "middle" << "ring" << "pinky" << fort::endr;
    header(t, 8);
    size_t row = 1;
    for(auto & hand : hands_)
    {
      if(!hand.present) continue;
      auto st = hand.state.GetData();
      auto cmd = hand.command.GetData();
      t << hand.side << "state [0-1]";
      for(size_t k = 0; k < g1::kNumHandMotors; ++k) { t << (st ? fmt::format("{:.2f}", (*st)[k]) : std::string("no data")); }
      t << fort::endr;
      for(size_t k = 0; k < g1::kNumHandMotors; ++k)
      {
        if(!st) paint(t, row, 2 + k, fort::color::red);
        // Finger far from its command (blocked or not following): yellow
        else if(cmd && std::abs((*st)[k] - (*cmd)[k]) > 0.15f) paint(t, row, 2 + k, fort::color::yellow);
      }
      ++row;
      t << "" << "command [0-1]";
      for(size_t k = 0; k < g1::kNumHandMotors; ++k) { t << (cmd ? fmt::format("{:.2f}", (*cmd)[k]) : std::string("-")); }
      t << fort::endr;
      ++row;
    }
    out += t.to_string();
  }
  return out;
}

void G1Control::DampingCommand(g1::MotorCommand & cmd, const g1::MotorState & ms)
{
  // Emergency damping, no Kp, only Kd with 0 ref vel
  for(size_t i = 0; i < n_joints_; i++)
  {
    auto motorId = jointIdsToMotorIds_[i];
    if(motorId < 0) continue;
    cmd.kp.at(motorId) = 0.f;
    cmd.kd.at(motorId) = kd_(i);
    cmd.q_ref.at(motorId) = ms.q.at(motorId);
    cmd.dq_ref.at(motorId) = 0.f;
    cmd.tau_ff.at(motorId) = 0.f;
  }
}

void G1Control::UnusedMotorsCommand(g1::MotorCommand & cmd, bool damping)
{
  // Motors not handled by mc_rtc (e.g. waist roll/pitch and wrists pitch/yaw when using the 23dof model on a 29dof
  // robot) are kept at their initial position. On a 23dof robot these motors do not exist and this is harmless.
  for(int m = 0; m < g1::kNumMotors; ++m)
  {
    if(!unusedMotor_[m]) continue;
    cmd.kp.at(m) = damping ? 0.f : kp_wait_motor_(m);
    cmd.kd.at(m) = kd_wait_motor_(m);
    cmd.q_ref.at(m) = q_hold_unused_[m];
    cmd.dq_ref.at(m) = 0.f;
    cmd.tau_ff.at(m) = 0.f;
  }
}

void G1Control::SetAdditionalSensors(const g1::MotorState & ms)
{
  auto & gc = mc_controller_->controller();

  /* Pelvis IMU copied to the FloatingBase body sensor (see the "floating-base-sensor" option) */
  if(!floating_base_sensor_.empty())
  {
    gc.setSensorOrientations({{floating_base_sensor_, Eigen::Quaterniond(mc_rbdyn::rpyToMat(stateIn_.rpyIn_))}});
    gc.setSensorAngularVelocities({{floating_base_sensor_, stateIn_.rateIn_}});
    gc.setSensorLinearAccelerations({{floating_base_sensor_, stateIn_.accIn_}});
  }

  /* Torso IMU (rt/secondary_imu), same conventions as the main IMU in MCControlUnitree2::run */
  if(!torso_imu_sensor_.empty())
  {
    auto torso_ptr = imu_torso_buffer_.GetData();
    if(torso_ptr)
    {
      Eigen::Vector3d rpy(torso_ptr->rpy[0], torso_ptr->rpy[1], torso_ptr->rpy[2]);
      gc.setSensorOrientations({{torso_imu_sensor_, Eigen::Quaterniond(mc_rbdyn::rpyToMat(rpy))}});
      gc.setSensorAngularVelocities(
          {{torso_imu_sensor_, Eigen::Vector3d(torso_ptr->omega[0], torso_ptr->omega[1], torso_ptr->omega[2])}});
      gc.setSensorLinearAccelerations(
          {{torso_imu_sensor_, Eigen::Vector3d(torso_ptr->acc[0], torso_ptr->acc[1], torso_ptr->acc[2])}});
    }
    else if(!torso_imu_warned_)
    {
      torso_imu_warned_ = true;
      mc_rtc::log::warning("[mc_unitree] G1: no data received on {}, {} is not updated", g1::TOPIC_IMU_TORSO,
                           torso_imu_sensor_);
    }
  }

  /* Motor temperatures */
  std::map<std::string, double> temperatures;
  for(size_t i = 0; i < n_joints_; ++i)
  {
    if(jointIdsToMotorIds_[i] >= 0 && robot_->jointHasJointSensor(joint_names_[i]))
    {
      temperatures[joint_names_[i]] = ms.temperature.at(jointIdsToMotorIds_[i]);
    }
  }
  if(!temperatures.empty()) { gc.setJointMotorTemperatures(temperatures); }
}

void G1Control::Control()
{
  g1::MotorCommand motor_command_tmp;
  const std::shared_ptr<const g1::MotorState> ms_tmp_ptr = motor_state_buffer_.GetData();
  const std::shared_ptr<const g1::ImuState> pelvis_ptr = imu_pelvis_buffer_.GetData();

  if(!ms_tmp_ptr || !pelvis_ptr) return;

  if(!q_hold_unused_init_)
  {
    q_hold_unused_ = ms_tmp_ptr->q;
    q_hold_unused_init_ = true;
  }

  time_ += control_dt_;

  /* Joint sensors (ref joint order) */
  Eigen::VectorXf q_pos(n_joints_), q_vel(n_joints_);
  for(size_t i = 0; i < n_joints_; i++)
  {
    auto motorId = jointIdsToMotorIds_[i];
    if(motorId < 0)
    {
      const auto [h, k] = handMotorOfJoint_[i];
      if(h >= 0)
      {
        auto hs = hands_[h].state.GetData();
        if(hs)
        {
          // Revo2 hand motor: measured position (normalized by brainco_hand_service)
          const auto & hand = hands_[h];
          const double q = hand.q_min[k] + std::clamp((*hs)[k], 0.f, 1.f) * (hand.q_max[k] - hand.q_min[k]);
          stateIn_.qIn_[i] = q;
          stateIn_.dqIn_[i] = 0.0;
          stateIn_.tauIn_[i] = 0.0;
          q_pos[i] = q;
          q_vel[i] = 0.f;
          continue;
        }
      }
      // No motor (or no hand state yet): loop back the command
      stateIn_.qIn_[i] = cmdOut_.qOut_[i];
      stateIn_.dqIn_[i] = cmdOut_.dqOut_[i];
      stateIn_.tauIn_[i] = 0.0;
      q_pos[i] = cmdOut_.qOut_[i];
      q_vel[i] = cmdOut_.dqOut_[i];
      continue;
    }
    stateIn_.qIn_[i] = ms_tmp_ptr->q.at(motorId);
    stateIn_.dqIn_[i] = ms_tmp_ptr->dq.at(motorId);
    stateIn_.tauIn_[i] = ms_tmp_ptr->tau.at(motorId);

    q_pos[i] = ms_tmp_ptr->q.at(motorId);
    q_vel[i] = ms_tmp_ptr->dq.at(motorId);
  }

  /* Main IMU (pelvis), used for the default body sensor */
  for(int i = 0; i < 3; i++)
  {
    stateIn_.rpyIn_[i] = pelvis_ptr->rpy.at(i);
    stateIn_.accIn_[i] = pelvis_ptr->acc.at(i);
    stateIn_.rateIn_[i] = pelvis_ptr->omega.at(i);
  }
  // unitree quaternion is [w, x, y, z]
  stateIn_.quatIn_ =
      Eigen::Quaterniond(pelvis_ptr->quat[0], pelvis_ptr->quat[1], pelvis_ptr->quat[2], pelvis_ptr->quat[3]);

  // Check if joints are out of position/velocity limits
  const bool lim_lower = ((q_pos - q_lim_lower_).array() < 0.0).any();
  const bool lim_upper = ((q_pos - q_lim_upper_).array() > 0.0).any();
  const bool lim_velocity_lower = ((q_vel - q_dot_lim_lower_).array() < 0.0).any();
  const bool lim_velocity_upper = ((q_vel - q_dot_lim_upper_).array() > 0.0).any();

  // Report motor errors on controlled motors (only when the error code changes)
  for(size_t i = 0; i < n_joints_; ++i)
  {
    auto motorId = jointIdsToMotorIds_[i];
    if(motorId < 0) continue;
    auto err = ms_tmp_ptr->error.at(motorId);
    if(err != last_motor_error_[motorId])
    {
      if(err != 0) { mc_rtc::log::error("[mc_unitree] G1: motor {} ({}) error code {}", motorId, joint_names_[i], err); }
      else { mc_rtc::log::info("[mc_unitree] G1: motor {} ({}) error cleared", motorId, joint_names_[i]); }
      last_motor_error_[motorId] = err;
    }
  }

  // Wrong robot model for this hardware revision: do not control the robot
  if(mode_machine_mismatch_ && status_ != STATUS_DAMPING)
  {
    SetEvent(fmt::format("DAMPING: the robot reports mode_machine {} but the model is for {}, use G1_mode_{}",
                         unsigned(mode_machine_), expected_mode_machine_, unsigned(mode_machine_)));
    mc_rtc::log::error("[mc_unitree] G1: robot model / hardware mismatch, switching to damping (set "
                       "'check-mode-machine: false' to bypass this check)");
    status_ = STATUS_DAMPING;
  }

  // Switch to waiting after initialization
  if((status_ == STATUS_INIT) && (time_ > init_duration_))
  {
    status_ = STATUS_WAITING_AIR;
    std::thread wait_thread(waiting, this);
    wait_thread.detach();
  }

  switch(status_)
  {
    case STATUS_RUN:
    {
      // If limits are breached, go to damping mode
      if(lim_lower || lim_upper || lim_velocity_lower || lim_velocity_upper)
      {
        const size_t n = n_joints_;
        {
          std::string joints;
          for(size_t i = 0; i < n; ++i)
          {
            if(q_pos[i] < q_lim_lower_[i] || q_pos[i] > q_lim_upper_[i] || q_vel[i] < q_dot_lim_lower_[i]
               || q_vel[i] > q_dot_lim_upper_[i])
            {
              joints += (joints.empty() ? "" : ", ") + joint_names_[i];
            }
          }
          SetEvent("DAMPING: joint " + std::string(lim_lower || lim_upper ? "position" : "velocity")
                   + " limit breached: " + joints);
        }
        if(lim_lower)
        {
          mc_rtc::log::error("[mc_unitree] Joint lower position limit breached!");
          for(size_t i = 0; i < n; ++i)
          {
            if(q_pos[i] < q_lim_lower_[i])
            {
              mc_rtc::log::error("  - {}: pos = {}, lower limit = {}", joint_names_[i], q_pos[i], q_lim_lower_[i]);
            }
          }
        }

        if(lim_upper)
        {
          mc_rtc::log::error("[mc_unitree] Joint upper position limit breached!");
          for(size_t i = 0; i < n; ++i)
          {
            if(q_pos[i] > q_lim_upper_[i])
            {
              mc_rtc::log::error("  - {}: pos = {}, upper limit = {}", joint_names_[i], q_pos[i], q_lim_upper_[i]);
            }
          }
        }

        if(lim_velocity_lower)
        {
          mc_rtc::log::error("[mc_unitree] Joint lower velocity limit breached!");
          for(size_t i = 0; i < n; ++i)
          {
            if(q_vel[i] < q_dot_lim_lower_[i])
            {
              mc_rtc::log::error("  - {}: vel = {}, lower vel limit = {}", joint_names_[i], q_vel[i],
                                 q_dot_lim_lower_[i]);
            }
          }
        }

        if(lim_velocity_upper)
        {
          mc_rtc::log::error("[mc_unitree] Joint upper velocity limit breached!");
          for(size_t i = 0; i < n; ++i)
          {
            if(q_vel[i] > q_dot_lim_upper_[i])
            {
              mc_rtc::log::error("  - {}: vel = {}, upper vel limit = {}", joint_names_[i], q_vel[i],
                                 q_dot_lim_upper_[i]);
            }
          }
        }

        status_ = STATUS_DAMPING;
        DampingCommand(motor_command_tmp, *ms_tmp_ptr);
        UnusedMotorsCommand(motor_command_tmp, true);
        break;
      }

      time_run_ += control_dt_;

      SetAdditionalSensors(*ms_tmp_ptr);
      mc_controller_->run(stateIn_, cmdOut_);
      UpdateHandCommands();

      // Send commands to the robot
      const ControlMode mode = mc_controller_->mode();
      for(size_t i = 0; i < n_joints_; ++i)
      {
        auto motorId = jointIdsToMotorIds_[i];
        if(motorId < 0) continue;
        switch(mode)
        {
          case ControlMode::Position:
          {
            motor_command_tmp.kp.at(motorId) = cmdOut_.kpOut_[i];
            motor_command_tmp.kd.at(motorId) = cmdOut_.kdOut_[i];
            motor_command_tmp.q_ref.at(motorId) = cmdOut_.qOut_[i];
            motor_command_tmp.dq_ref.at(motorId) = cmdOut_.dqOut_[i];
            motor_command_tmp.tau_ff.at(motorId) = 0.f;
            break;
          }

          case ControlMode::Velocity:
          {
            motor_command_tmp.kp.at(motorId) = 0.f;
            motor_command_tmp.kd.at(motorId) = cmdOut_.kdOut_[i];
            motor_command_tmp.q_ref.at(motorId) = 0.f;
            motor_command_tmp.dq_ref.at(motorId) = cmdOut_.dqOut_[i];
            motor_command_tmp.tau_ff.at(motorId) = 0.f;
            break;
          }

          case ControlMode::Torque:
          {
            // ---------------------------------------------------------------------------------------------
            // Option A (default): torque control by doing an inverse PD command
            //   tau_motor = kp (q_ref - q) + kd (0 - dq) = tau  with  q_ref = q + (1/kp) * (tau + kd * dq)
            // ---------------------------------------------------------------------------------------------
            motor_command_tmp.kp.at(motorId) = kp_torque_[i];
            motor_command_tmp.kd.at(motorId) = kd_torque_[i];
            motor_command_tmp.q_ref.at(motorId) =
                ms_tmp_ptr->q.at(motorId)
                + (1.0f / kp_torque_[i]) * (cmdOut_.tauOut_[i] + kd_torque_[i] * ms_tmp_ptr->dq.at(motorId));
            motor_command_tmp.dq_ref.at(motorId) = 0.f;
            motor_command_tmp.tau_ff.at(motorId) = 0.f;

            // ---------------------------------------------------------------------------------------------
            // Option B: direct torque command (feedforward torque only, no PD in the motor driver)
            //   To test it: comment Option A above and uncomment the block below.
            // ---------------------------------------------------------------------------------------------
            // motor_command_tmp.kp.at(motorId) = 0.f;
            // motor_command_tmp.kd.at(motorId) = 0.f;
            // motor_command_tmp.q_ref.at(motorId) = ms_tmp_ptr->q.at(motorId);
            // motor_command_tmp.dq_ref.at(motorId) = 0.f;
            // motor_command_tmp.tau_ff.at(motorId) = cmdOut_.tauOut_[i];
            break;
          }

          default:
          {
            mc_rtc::log::error("[mc_unitree] Unknown control mode!");
            SetEvent("DAMPING: unknown control mode");
            status_ = STATUS_DAMPING;
            break;
          }
        }
      }
      if(status_ == STATUS_DAMPING) { DampingCommand(motor_command_tmp, *ms_tmp_ptr); }
      UnusedMotorsCommand(motor_command_tmp, status_ == STATUS_DAMPING);
      break;
    }

    case STATUS_WAITING_AIR:
    case STATUS_WAITING_GRD:
    {
      // Wait at default configuration
      for(size_t i = 0; i < n_joints_; ++i)
      {
        auto motorId = jointIdsToMotorIds_[i];
        if(motorId < 0) continue;
        motor_command_tmp.kp.at(motorId) = kp_wait_(i);
        motor_command_tmp.kd.at(motorId) = kd_wait_(i);
        motor_command_tmp.q_ref.at(motorId) = q_init_(i);
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = 0.f;
      }
      UnusedMotorsCommand(motor_command_tmp, false);
      break;
    }

    case STATUS_GAIN_TRANSITION:
    {
      // Interpolation from waiting gains to controller gains
      time_run_ += control_dt_;

      float alpha = std::clamp(time_run_ / interp_duration_, 0.f, 1.f);
      for(size_t i = 0; i < n_joints_; ++i)
      {
        auto motorId = jointIdsToMotorIds_[i];
        if(motorId < 0) continue;
        motor_command_tmp.kp.at(motorId) = kp_wait_(i) * (1 - alpha) + kp_(i) * alpha;
        motor_command_tmp.kd.at(motorId) = kd_wait_(i) * (1 - alpha) + kd_(i) * alpha;
        motor_command_tmp.q_ref.at(motorId) = q_init_(i);
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = 0.f;
      }
      UnusedMotorsCommand(motor_command_tmp, false);

      // If transition is over, switch to the controller
      if(time_run_ >= interp_duration_)
      {
        time_run_ = -control_dt_;
        status_ = STATUS_RUN;
      }
      break;
    }

    case STATUS_INIT:
    {
      // Slowly move to default configuration
      float ratio = std::clamp(time_, 0.f, init_duration_) / init_duration_;
      for(size_t i = 0; i < n_joints_; ++i)
      {
        auto motorId = jointIdsToMotorIds_[i];
        if(motorId < 0) continue;
        motor_command_tmp.kp.at(motorId) = kp_wait_(i);
        motor_command_tmp.kd.at(motorId) = kd_wait_(i);
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = 0.f;

        float q_des = (q_init_(i) - ms_tmp_ptr->q.at(motorId)) * ratio + ms_tmp_ptr->q.at(motorId);
        motor_command_tmp.q_ref.at(motorId) = q_des;
      }
      UnusedMotorsCommand(motor_command_tmp, false);
      break;
    }

    default:
    { // case STATUS_DAMPING:
      DampingCommand(motor_command_tmp, *ms_tmp_ptr);
      UnusedMotorsCommand(motor_command_tmp, true);
    }
  }
  // Write to command buffer
  motor_command_buffer_.SetData(motor_command_tmp);
  // Publish immediately: lowest delay between the state used by the controller and the command
  if(lowcmd_publisher_) { LowCommandWriter(); }
}

void G1Control::SetupHands(const mc_rtc::Configuration & config)
{
  handMotorOfJoint_.assign(n_joints_, {-1, -1});
  bool enabled = true;
  config("revo2-hands", enabled);
  double hand_speed = hand_speed_; // mc_rtc::Configuration reads double, not float
  config("hand-speed", hand_speed);
  hand_speed_ = static_cast<float>(std::clamp(hand_speed, 0.0, 1.0));
  const std::array<std::string, 2> sides = {"left", "right"};
  for(size_t h = 0; h < 2; ++h)
  {
    auto & hand = hands_[h];
    hand.side = sides[h];
    const auto names = g1::revo2MotorJoints(hand.side);
    size_t found = 0;
    for(size_t k = 0; k < names.size(); ++k)
    {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), names[k]);
      if(it == joint_names_.end()) continue;
      const int i = static_cast<int>(std::distance(joint_names_.begin(), it));
      hand.joint[k] = i;
      hand.q_min[k] = q_lim_lower_(i);
      hand.q_max[k] = q_lim_upper_(i);
      ++found;
    }
    if(found == 0) continue;
    if(found != names.size())
    {
      mc_rtc::log::warning("[mc_unitree] G1: {} Revo2 hand found with only {}/{} motor joints, not controlled",
                           hand.side, found, names.size());
      continue;
    }
    hand.present = true;
    if(enabled)
    {
      for(size_t k = 0; k < names.size(); ++k) { handMotorOfJoint_[hand.joint[k]] = {static_cast<int>(h), static_cast<int>(k)}; }
      mc_rtc::log::info("[mc_unitree] G1: {} Revo2 hand: 6 motors, joint ranges mapped to [0, 1]", hand.side);
    }
  }
  hands_enabled_ = enabled && (hands_[0].present || hands_[1].present);
  if(!enabled && (hands_[0].present || hands_[1].present))
  {
    mc_rtc::log::info("[mc_unitree] G1: Revo2 hands present in the model but not controlled (revo2-hands: false)");
  }
}

void G1Control::UpdateHandCommands()
{
  if(!hands_enabled_) return;
  for(auto & hand : hands_)
  {
    if(!hand.present) continue;
    g1::HandArray cmd;
    for(size_t k = 0; k < cmd.size(); ++k)
    {
      const float range = hand.q_max[k] - hand.q_min[k];
      const float q = static_cast<float>(cmdOut_.qOut_[hand.joint[k]]);
      cmd[k] = range > 0.f ? std::clamp((q - hand.q_min[k]) / range, 0.f, 1.f) : 0.f;
    }
    hand.command.SetData(cmd);
  }
}

void G1Control::HandCommandWriter()
{
  for(auto & hand : hands_)
  {
    if(!hand.present || !hand.publisher) continue;
    auto cmd = hand.command.GetData();
    if(!cmd) continue; // nothing sent before the controller runs, the hand keeps its position
    unitree_go::msg::dds_::MotorCmds_ msg;
    msg.cmds().resize(g1::kNumHandMotors);
    for(size_t k = 0; k < g1::kNumHandMotors; ++k)
    {
      msg.cmds()[k].q() = (*cmd)[k];
      msg.cmds()[k].dq() = hand_speed_;
    }
    hand.publisher->Write(msg);
  }
}

void G1Control::HandStateHandler(size_t h, const void * message)
{
  const auto & msg = *static_cast<const unitree_go::msg::dds_::MotorStates_ *>(message);
  auto & hand = hands_[h];
  if(msg.states().size() < g1::kNumHandMotors)
  {
    if(!hand.state_warned)
    {
      hand.state_warned = true;
      mc_rtc::log::warning("[mc_unitree] G1: {} hand state has {} motors instead of {}", hand.side,
                           msg.states().size(), g1::kNumHandMotors);
    }
    return;
  }
  g1::HandArray q;
  for(size_t k = 0; k < g1::kNumHandMotors; ++k) { q[k] = msg.states()[k].q(); }
  if(!hand.state.GetData()) { mc_rtc::log::success("[mc_unitree] G1: receiving the {} Revo2 hand state", hand.side); }
  hand.state.SetData(q);
}

/**
 * @brief Set initial state values for simulation
 *
 * @param stance Value defined by RobotModule
 */
void G1Control::setInitialState(const std::map<std::string, std::vector<double>> & stance)
{
  /* Start stance */
  for(size_t i = 0; i < n_joints_; i++)
  {
    const std::string & jname = joint_names_[i];
    if(stance.count(jname)) { stateIn_.qIn_[i] = stance.at(jname)[0]; }
    else { stateIn_.qIn_[i] = 0.0; }
    stateIn_.dqIn_[i] = 0.0;
    stateIn_.tauIn_[i] = 0.0;
  }

  /* Set body(imu) sensor values */
  stateIn_.rpyIn_.setZero();
  stateIn_.quatIn_.setIdentity();
  stateIn_.accIn_.setZero();
  stateIn_.rateIn_.setZero();
}

/**
 * @brief Loop back the value of "data" to "state"
 *
 * @param data Command data for sending to G1 robot
 */
void G1Control::loopbackState(const G1CommandData & data)
{
  /*  Set current sensor values */
  for(size_t i = 0; i < n_joints_; i++)
  {
    stateIn_.qIn_[i] = data.qOut_[i];
    stateIn_.dqIn_[i] = data.dqOut_[i];
    stateIn_.tauIn_[i] = data.tauOut_[i];

    stateIn_.qIn_[i] = std::clamp<double>(stateIn_.qIn_[i], q_lim_lower_[i], q_lim_upper_[i]);
  }
}
