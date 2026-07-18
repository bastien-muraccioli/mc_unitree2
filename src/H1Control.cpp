
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include "H1Control.h"

using namespace mc_unitree;

/**
 * @brief Interface constructor and destructor
 * DDS connection with robot using specified parameters.
 * Control level is set to LOW-level.
 *
 * @param config_param Configuration file parameters
 */
H1Control::H1Control(MCControlUnitree2<H1Control, H1SensorInfo, H1CommandData, H1ConfigParameter> * mc_controller, mc_rbdyn::Robot * robot, const H1ConfigParameter & config_param)
  : mc_controller_(mc_controller), robot_(robot), control_dt_(mc_controller->controller().timestep()), config_(config_param)
{
  mc_rtc::log::info("[mc_unitree] H1 Number of joints: {}", robot_->refJointOrder().size());
  mc_rtc::log::info("[mc_unitree] H1 Control dt: {}", control_dt_);

  mc_control::MCGlobalController::GlobalConfiguration gconfig("", nullptr);
  if(!gconfig.config.has("Unitree"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_unitree] Missing Unitree configuration");
  }
  auto unitree_config = gconfig.config("Unitree");
  if(!unitree_config.has("h1"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_unitree] Missing Unitree H1 configuration");
  }
  auto h1_config = unitree_config("h1");
  
  // q_init, q_lim_lower, q_lim_upper will be overwritten by definition in mc_h1 and urdf
  if(h1_config.has("q_init"))
  {
    Eigen::VectorXd q_init = h1_config("q_init");
    if(q_init.size() != 20) q_init_ = config_param.q_init_;
    else q_init_ = q_init.cast<float>();
  }
  else q_init_ = config_param.q_init_;

  if(h1_config.has("q_lim_lower"))
  {
    Eigen::VectorXd q_lim_lower = h1_config("q_lim_lower");
    if(q_lim_lower.size() != 20) q_lim_lower_ = config_param.q_lim_lower_;
    else q_lim_lower_ = q_lim_lower.cast<float>();
  }
  else q_lim_lower_ = config_param.q_lim_lower_;

  if(h1_config.has("q_lim_upper"))
  {
    Eigen::VectorXd q_lim_upper = h1_config("q_lim_upper");
    if(q_lim_upper.size() != 20) q_lim_upper_ = config_param.q_lim_upper_;
    else q_lim_upper_ = q_lim_upper.cast<float>();
  }
  else q_lim_upper_ = config_param.q_lim_upper_;

  if(h1_config.has("qdot_lim"))
  {
    Eigen::VectorXd qdot_lim_upper = h1_config("qdot_lim");
    Eigen::VectorXd qdot_lim_lower = -1.0 * qdot_lim_upper;
    if(qdot_lim_upper.size() != 20){
      q_dot_lim_upper_ = config_param.qdot_lim_;
      q_dot_lim_lower_ = -1.0 * config_param.qdot_lim_;
    }
    else{
      q_dot_lim_upper_ = qdot_lim_upper.cast<float>();
      q_dot_lim_lower_ = qdot_lim_lower.cast<float>();
    }
  }
  else {
    q_dot_lim_lower_ = -1.0 * config_param.qdot_lim_;
    q_dot_lim_upper_ = config_param.qdot_lim_;
  }

  if(h1_config.has("kp"))
  {
    Eigen::VectorXd kp = h1_config("kp");
    if(kp.size() != 20) kp_ = config_param.kp_;
    else kp_ = kp.cast<float>();
  }
  else kp_ = config_param.kp_;

  if(h1_config.has("kd"))
  {
    Eigen::VectorXd kd = h1_config("kd");
    if(kd.size() != 20) kd_ = config_param.kd_;
    else kd_ = kd.cast<float>();
  }
  else kd_ = config_param.kd_;

  if(h1_config.has("kp_torque"))
  {
    Eigen::VectorXd kp_torque = h1_config("kp_torque");
    if(kp_torque.size() != 20) kp_torque_ = config_param.kp_torque_;
    else kp_torque_ = kp_torque.cast<float>();
  }
  else kp_torque_ = config_param.kp_torque_;

  if(h1_config.has("kd_torque"))
  {
    Eigen::VectorXd kd_torque = h1_config("kd_torque");
    if(kd_torque.size() != 20) kd_torque_ = config_param.kd_torque_;
    else kd_torque_ = kd_torque.cast<float>();
  }
  else kd_torque_ = config_param.kd_torque_;

  if(h1_config.has("kp_wait"))
  {
    Eigen::VectorXd kp_wait = h1_config("kp_wait");
    if(kp_wait.size() != 20) kp_wait_ = config_param.kp_stand_;
    else kp_wait_ = kp_wait.cast<float>();
  }
  else kp_wait_ = config_param.kp_stand_;

  if(h1_config.has("kd_wait"))
  {
    Eigen::VectorXd kd_wait = h1_config("kd_wait");
    if(kd_wait.size() != 20) kd_wait_ = config_param.kd_stand_;
    else kd_wait_ = kd_wait.cast<float>();
  }
  else kd_wait_ = config_param.kd_stand_;

  if(h1_config.has("tau_ff"))
  {
    Eigen::VectorXd tau_ff = h1_config("tau_ff");
    if(tau_ff.size() != 20) tau_ff_ = config_param.tau_ff_;
    else tau_ff_ = tau_ff.cast<float>();
  }
  else tau_ff_ = config_param.tau_ff_;

  stateIn_.qIn_.resize(robot->refJointOrder().size(), 0.0);
  stateIn_.dqIn_.resize(robot->refJointOrder().size(), 0.0);
  stateIn_.tauIn_.resize(robot->refJointOrder().size(), 0.0);
  stateIn_.rpyIn_.setZero();
  stateIn_.quatIn_.setIdentity();
  stateIn_.accIn_.setZero();
  stateIn_.rateIn_.setZero();
  
  cmdOut_.qOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.dqOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.tauOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.kpOut_.resize(robot->refJointOrder().size());
  cmdOut_.kdOut_.resize(robot->refJointOrder().size());
  cmdOut_.kpOutTorque_.resize(robot->refJointOrder().size());
  cmdOut_.kdOutTorque_.resize(robot->refJointOrder().size());
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
  {
    cmdOut_.kpOut_[i] = kp_[i];
    cmdOut_.kdOut_[i] = kd_[i];
    cmdOut_.kpOutTorque_[i] = kp_torque_[i];
    cmdOut_.kdOutTorque_[i] = kd_torque_[i];
  }
  
  refJointOrderToMCJointId_.resize(kNumMotors, -1);
  for (size_t i = 0 ; i < robot->refJointOrder().size() ; i++)
  {
    const std::string & jname = robot_->refJointOrder()[i];
    auto mcJointId = robot->jointIndexByName(jname);
    if (robot->mbc().q[mcJointId].empty())
      continue;
    
    refJointOrderToMCJointId_[i] = mcJointId;
    mcJointIdToJointId_[mcJointId] = i;
    /* Overrite initial q_init_ if stance is set and not provided by the config file */
    if(robot->stance().count(jname) && !h1_config.has("q_init"))
    {
      q_init_(i) = robot->stance().at(jname)[0];
    }
  }

  mode_ = config_param.mode_;

  const std::string &network = config_param.network_;

  if (!network.empty())
  {
    int simulation = 1;
    if(network != "lo") simulation = 0;
    mc_rtc::log::info("[mc_unitree] H1Control: Using network setting: {}", network);

    /* Initialize */
    unitree::robot::ChannelFactory::Instance()->Init(simulation, network);
    mc_rtc::log::info("Initialize channel factory.");
    
    lowcmd_publisher_.reset(
      new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();
    command_writer_ptr_ = unitree::common::CreateRecurrentThreadEx(
      "command_writer", UT_CPU_ID_NONE, 2000, &H1Control::LowCommandWriter, this);
  
    lowstate_subscriber_.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(
      std::bind(&H1Control::LowStateHandler, this, std::placeholders::_1),
      1);
    
#if defined(__ENABLE_RT_PREEMPT__)
    pthread_create(&control_thread_, NULL,
                   [](void* arg) -> void* {
                     auto* ctrl = static_cast<H1Control::Control*>(arg);
                     ctrl->Control();
                     return nullptr;
                   }, NULL);
#else
    int control_period_us = control_dt_ * 1e6;
    control_thread_ptr_ = unitree::common::CreateRecurrentThreadEx(
      "control", UT_CPU_ID_NONE, control_period_us, &H1Control::Control,
      this);
#endif
    
    int report_period_us = report_dt_ * 1e6;
    report_sensor_ptr_ = unitree::common::CreateRecurrentThreadEx(
      "report_sensor", UT_CPU_ID_NONE, report_period_us,
      &H1Control::UpdateTables, this, false);
    
    // Initialize tables for console display
    UpdateTables(true);
  }
}

H1Control::~H1Control()
{
#if defined(__ENABLE_RT_PREEMPT__)
  pthread_join(control_thread_, NULL);
#endif
}

////
// WAITING BEFORE LAUNCHING CONTROLLER
////

// Wait for Enter key press
void waiting(H1Control *controller)
{
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1000ms);
  std::cin.get();
  controller->endWaiting();
}

void H1Control::endWaiting()
{
  if (status_ == STATUS_WAITING_AIR)
  {
    status_ = STATUS_WAITING_GRD;
    std::thread wait_thread(waiting, this);
    wait_thread.detach();
  }
  else if (status_ == STATUS_WAITING_GRD)
  {
    time_run_ = -control_dt_;
    status_ = STATUS_GAIN_TRANSITION;
  }
}

void H1Control::LowCommandWriter()
{
  unitree_go::msg::dds_::LowCmd_ dds_low_command{};
  dds_low_command.head()[0] = 0xFE;
  dds_low_command.head()[1] = 0xEF;
  dds_low_command.level_flag() = 0xFF;
  dds_low_command.gpio() = 0;
  
  const std::shared_ptr<const MotorCommand> mc_tmp_ptr =
    motor_command_buffer_.GetData();
  if (mc_tmp_ptr)
  {
    for (int i = 0; i < kNumMotors; ++i)
    {
      if (IsWeakMotor(i))
      {
        dds_low_command.motor_cmd().at(i).mode() = (0x01);
      }
      else
      {
        dds_low_command.motor_cmd().at(i).mode() = (0x0A);
      }
      dds_low_command.motor_cmd().at(i).tau() = mc_tmp_ptr->tau_ff.at(i);
      dds_low_command.motor_cmd().at(i).q() = mc_tmp_ptr->q_ref.at(i);
      dds_low_command.motor_cmd().at(i).dq() = mc_tmp_ptr->dq_ref.at(i);
      dds_low_command.motor_cmd().at(i).kp() = mc_tmp_ptr->kp.at(i);
      dds_low_command.motor_cmd().at(i).kd() = mc_tmp_ptr->kd.at(i);
    }
    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command,
                                      (sizeof(dds_low_command) >> 2) - 1);
    lowcmd_publisher_->Write(dds_low_command);
  }
}

void H1Control::LowStateHandler(const void *message)
{
  unitree_go::msg::dds_::LowState_ low_state =
    *(unitree_go::msg::dds_::LowState_ *)message;
  
  RecordMotorState(low_state);
  RecordBaseState(low_state);
}

void H1Control::RecordMotorState(const unitree_go::msg::dds_::LowState_ &msg)
{
  MotorState ms_tmp;
  for (int i = 0; i < kNumMotors; ++i)
  {
    ms_tmp.q.at(i) = msg.motor_state()[i].q();
    ms_tmp.dq.at(i) = msg.motor_state()[i].dq();
    ms_tmp.tau.at(i) = msg.motor_state()[i].tau_est();
  }
  
  motor_state_buffer_.SetData(ms_tmp);
}

void H1Control::RecordBaseState(const unitree_go::msg::dds_::LowState_ &msg)
{
  BaseState bs_tmp;
  bs_tmp.omega = msg.imu_state().gyroscope();
  bs_tmp.quat = msg.imu_state().quaternion();
  bs_tmp.rpy = msg.imu_state().rpy();
  bs_tmp.acc = msg.imu_state().accelerometer();
  
  base_state_buffer_.SetData(bs_tmp);
}

void H1Control::ReportSensors()
{
  const std::shared_ptr<const BaseState> bs_tmp_ptr =
    base_state_buffer_.GetData();
  const std::shared_ptr<const MotorState> ms_tmp_ptr =
    motor_state_buffer_.GetData();
  if (bs_tmp_ptr)
  {
    mc_rtc::log::info("Base RPY: [{:.4f}, {:.4f}, {:.4f}]",
                      bs_tmp_ptr->rpy.at(0),
                      bs_tmp_ptr->rpy.at(1),
                      bs_tmp_ptr->rpy.at(2));

    mc_rtc::log::info("Gyro: [{:.4f}, {:.4f}, {:.4f}]",
                      bs_tmp_ptr->omega.at(0),
                      bs_tmp_ptr->omega.at(1),
                      bs_tmp_ptr->omega.at(2));
    mc_rtc::log::info("Accelerometer: [{:.4f}, {:.4f}, {:.4f}]",
                      bs_tmp_ptr->acc.at(0),
                      bs_tmp_ptr->acc.at(1),
                      bs_tmp_ptr->acc.at(2));
  }
  if (ms_tmp_ptr)
  {
    mc_rtc::log::info("Joint positions: [");
    for (size_t i = 0; i < robot_->refJointOrder().size(); ++i)
    {
      mc_rtc::log::info("{:.4f}, ", ms_tmp_ptr->q.at(jointIdsToMotorIds[i]));
    }
    mc_rtc::log::info("]");
    mc_rtc::log::info("Joint velocities: [");
    for (size_t i = 0; i < robot_->refJointOrder().size(); ++i)
    {
      mc_rtc::log::info("{:.4f}, ", ms_tmp_ptr->dq.at(jointIdsToMotorIds[i]));
    }
    mc_rtc::log::info("]");
  }
}

void H1Control::UpdateTables(bool init)
{
  if(status_ != prev_status_)
  {
    prev_status_ = status_;
    switch (status_)
    {
    case STATUS_INIT:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃      Initialization      ┃");
      mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
      break;
    case STATUS_WAITING_AIR:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃    Waiting in the air    ┃");
      mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
      break;
    case STATUS_WAITING_GRD:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃   Waiting on the ground  ┃");
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

void H1Control::Control()
{
#if defined(__ENABLE_RT_PREEMPT__)
  if (set_sched_prio(RT_PRIO_MAX-2, TASK_PERIOD) == -1)
  {
    perror("set_sched_prio failed on MCControlUnitree2.\n");
    return;
  } /* in art_process */
#endif
  
  MotorCommand motor_command_tmp;
  const std::shared_ptr<const MotorState> ms_tmp_ptr =
    motor_state_buffer_.GetData();
  const std::shared_ptr<const BaseState> bs_tmp_ptr =
    base_state_buffer_.GetData();
  
  if (!ms_tmp_ptr || !bs_tmp_ptr)
    return;
  
  time_ += control_dt_;
  
  Vector20 q_pos, q_vel;
  for (size_t i = 0 ; i < robot_->refJointOrder().size(); i++)
  {
    auto motorId = jointIdsToMotorIds[i];
    stateIn_.qIn_[i] = ms_tmp_ptr->q.at(motorId);
    stateIn_.dqIn_[i] = ms_tmp_ptr->dq.at(motorId);
    stateIn_.tauIn_[i] = ms_tmp_ptr->tau.at(motorId);
    
    q_pos[i] = ms_tmp_ptr->q.at(motorId);
    q_vel[i] = ms_tmp_ptr->dq.at(motorId);
  }
  q_pos[robot_->refJointOrder().size()] = 0.0;
  q_vel[robot_->refJointOrder().size()] = 0.0;
  
  for (int i = 0 ; i < 3 ; i++)
  {
    stateIn_.rpyIn_[i] = bs_tmp_ptr->rpy.at(i);
    stateIn_.accIn_[i] = bs_tmp_ptr->acc.at(i);
    stateIn_.rateIn_[i] = bs_tmp_ptr->omega.at(i);
  }
  for (int i = 0 ; i < 4 ; i++)
  {
    stateIn_.quatIn_.coeffs()[i] = bs_tmp_ptr->quat.at(i);
  }
  
  // Check if joints are too close from position limits
  const bool lim_lower = ((q_pos - q_lim_lower_).array() < 0.0).any();
  const bool lim_upper = ((q_pos - q_lim_upper_).array() > 0.0).any();
  const bool lim_velocity_lower = ((q_vel - q_dot_lim_lower_).array() < 0.0).any();
  const bool lim_velocity_upper = ((q_vel - q_dot_lim_upper_).array() > 0.0).any();
  
  // Switch to waiting after initialization
  if ((status_ == STATUS_INIT) && (time_ > init_duration_))
  {
    status_ = STATUS_WAITING_AIR;
    std::thread wait_thread(waiting, this);
    wait_thread.detach();
  }
  
  switch (status_)
  {
  case STATUS_RUN:
  {
    // If limits are breached, go to damping mode
    if (lim_lower || lim_upper || lim_velocity_lower || lim_velocity_upper)
    {

      const int n = joint_names_.size();
      if (lim_lower)
      {
        mc_rtc::log::error("[mc_unitree] Joint lower position limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_pos[i] < q_lim_lower_[i])
          {
            mc_rtc::log::error("  - {}: pos = {}, lower limit = {}", 
                              joint_names_[i], q_pos[i], q_lim_lower_[i]);
          }
        }
      }

      if (lim_upper)
      {
        mc_rtc::log::error("[mc_unitree] Joint upper position limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_pos[i] > q_lim_upper_[i])
          {
            mc_rtc::log::error("  - {}: pos = {}, upper limit = {}", 
                              joint_names_[i], q_pos[i], q_lim_upper_[i]);
          }
        }
      }

      if (lim_velocity_lower)
      {
        mc_rtc::log::error("[mc_unitree] Joint lower velocity limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_vel[i] < q_dot_lim_lower_[i])
          {
            mc_rtc::log::error("  - {}: vel = {}, lower vel limit = {}", 
                              joint_names_[i], q_vel[i], q_dot_lim_lower_[i]);
          }
        }
      }

      if (lim_velocity_upper)
      {
        mc_rtc::log::error("[mc_unitree] Joint upper velocity limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_vel[i] > q_dot_lim_upper_[i])
          {
            mc_rtc::log::error("  - {}: vel = {}, upper vel limit = {}", 
                              joint_names_[i], q_vel[i], q_dot_lim_upper_[i]);
          }
        }
      }

      status_ = STATUS_DAMPING;
    }

    auto &datastore = mc_controller_->controller().controller().datastore();
    if (datastore.has("ControlMode"))
    {
      setControlMode(datastore.get<std::string>("ControlMode"));
    }

    time_run_ += control_dt_;

    mc_controller_->run(stateIn_, cmdOut_);
    
    // Send commands to the robot
    if(mode_ == ControlMode::Position)
    {
      for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
      {
        auto motorId = jointIdsToMotorIds[i];
        motor_command_tmp.kp.at(motorId) = cmdOut_.kpOut_[i];
        motor_command_tmp.kd.at(motorId) = cmdOut_.kdOut_[i];
        motor_command_tmp.q_ref.at(motorId) = cmdOut_.qOut_[i];
        motor_command_tmp.dq_ref.at(motorId) = cmdOut_.dqOut_[i];
        motor_command_tmp.tau_ff.at(motorId) = 0.f;
      }
      break;
    }
    // Torque control by doing inverse PD command (More precise than adding torque feedforward): q_ref = q + Kp^-1 (tau_cmd + Kd * dq)
    else if(mode_ == ControlMode::Torque) 
    {
      auto q_ref = cmdOut_.qOut_;

      for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
      {
        auto motorId = jointIdsToMotorIds[i];
        motor_command_tmp.q_ref.at(motorId) = ms_tmp_ptr->q.at(motorId) + (1.0f / cmdOut_.kpOutTorque_[i]) * (cmdOut_.tauOut_[i] + cmdOut_.kdOutTorque_[i] * ms_tmp_ptr->dq.at(motorId));
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.kp.at(motorId) = cmdOut_.kpOutTorque_[i];
        motor_command_tmp.kd.at(motorId) = cmdOut_.kdOutTorque_[i];
        motor_command_tmp.tau_ff.at(motorId) = 0.f;
      }
      break;
    }
    mc_rtc::log::error("[mc_unitree] Unknown control mode!");
    status_ = STATUS_DAMPING;
    break;
  }
  
  case STATUS_WAITING_AIR:
  {
    // Wait at default configuration
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = kp_wait_(i);
      motor_command_tmp.kd.at(motorId) = kd_wait_(i);
      motor_command_tmp.q_ref.at(motorId) = q_init_(i);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
    break;
  }
  
  case STATUS_WAITING_GRD:
  {
    // Wait at default configuration
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = kp_wait_(i);
      motor_command_tmp.kd.at(motorId) = kd_wait_(i);
      motor_command_tmp.q_ref.at(motorId) = q_init_(i);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
    break;
  }
  
  case STATUS_GAIN_TRANSITION:
  {
    // Interpolation from waiting gains to policy gains
    time_run_ += control_dt_;
    
    // Slowly switch PD gains to policy gains
    float alpha = time_run_ / interp_duration_;
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = kp_wait_(i) * (1 - alpha) + kp_(i) * alpha;
      motor_command_tmp.kd.at(motorId) = kd_wait_(i) * (1 - alpha) + kd_(i) * alpha;
      motor_command_tmp.q_ref.at(motorId) = q_init_(i);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
    
    // If transition is over, switch to the policy
    if (time_run_ >= interp_duration_)
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
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = kp_wait_(i);
      motor_command_tmp.kd.at(motorId) = kd_wait_(i);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
      
      float q_des = (q_init_(i) - ms_tmp_ptr->q.at(motorId)) * ratio +
        ms_tmp_ptr->q.at(motorId);
      motor_command_tmp.q_ref.at(motorId) = q_des;
    }
    break;
  }
  
  default:
  { // case STATUS_DAMPING:
    // Emergency damping, no Kp, only Kd with 0 ref vel
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
    {
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = 0.f;
      motor_command_tmp.kd.at(motorId) = kd_(i);
      motor_command_tmp.q_ref.at(motorId) = ms_tmp_ptr->q.at(motorId);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
  }
  }
  // Write to command buffer
  motor_command_buffer_.SetData(motor_command_tmp);
  
  // Log sensors and commands
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
  {
    auto motorId = jointIdsToMotorIds[i];
    tau_des_[i] =
      motor_command_tmp.kp.at(motorId) *
      (motor_command_tmp.q_ref.at(motorId) - ms_tmp_ptr->q.at(motorId)) +
      motor_command_tmp.kd.at(motorId) *
      (motor_command_tmp.dq_ref.at(motorId) - ms_tmp_ptr->dq.at(motorId)) +
      motor_command_tmp.tau_ff.at(motorId);
  }
}

/**
 * @brief Set initial state values for simulation
 * 
 * @param stance Value defined by RobotModule
 * @param state Current sensor values information
 */
void H1Control::setInitialState(const std::map<std::string, std::vector<double>> & stance)
{
  /* Start stance */
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
  {
    const std::string & jname = robot_->refJointOrder()[i];
    auto mcJointId = robot_->jointIndexByName(jname);
    auto jointId = mcJointIdToJointId(mcJointId);
    
    if(stance.count(jname))
    {
      stateIn_.qIn_[jointId] = stance.at(jname)[0];
    }
    else
    {
      stateIn_.qIn_[jointId] = 0.0;
    }
    stateIn_.dqIn_[jointId] = 0.0;
    stateIn_.tauIn_[jointId] = 0.0;
  }
  
  /* Set body(imu) sensor values */
  stateIn_.rpyIn_.setZero();
  stateIn_.quatIn_.setIdentity();
  stateIn_.accIn_.setZero();
  stateIn_.rateIn_.setZero();
};

/**
 * @brief Loop back the value of "data" to "state"
 * 
 * @param data Command data for sending to H1 robot
 * @param state Current sensor values information
 */
void H1Control::loopbackState(const H1CommandData & data)
{
  /*  Set current sensor values */
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
  {
    stateIn_.qIn_[i] = data.qOut_[i];
    stateIn_.dqIn_[i] = data.dqOut_[i];
    stateIn_.tauIn_[i] = data.tauOut_[i];
    
    if (stateIn_.qIn_[i] > robot_->qu()[i][0])
      stateIn_.qIn_[i] = robot_->qu()[i][0];
    else if (stateIn_.qIn_[i] < robot_->ql()[i][0])
      stateIn_.qIn_[i] = robot_->ql()[i][0];
  }
}

void H1Control::setControlMode(const std::string &mode) {
  if (mode.compare("Position") == 0) {
    mode_ = ControlMode::Position;
    return;
  }
  else if (mode.compare("Torque") == 0) {
   mode_ = ControlMode::Torque;
   return;
  }
  else {
    mc_rtc::log::error("{} ControlMode not supported", mode);
  }
} 