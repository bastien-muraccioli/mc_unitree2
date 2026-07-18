#pragma once

#include <thread>

// SpaceVecAlg
#include <SpaceVecAlg/Conversions.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_control/mc_global_controller.h>
#include <condition_variable>

#include "ControlMode.h"

namespace mc_unitree
{
/**
 * @brief mc_rtc control interface for unitree robots
 */
template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
class MCControlUnitree2
{
public:
  /**
   * @brief Interface constructor and destructor
   */
  MCControlUnitree2(mc_control::MCGlobalController & controller, const std::string & network);
  
  virtual ~MCControlUnitree2();

  /**
   * @brief Return a reference to the global mc_rtc controller
   */
  mc_control::MCGlobalController & controller()
  {
    return globalController_;
  }
  
  void run(const RobotSensorInfo & state, RobotCommandData & cmdData);
  
  bool getServoGains(std::vector<double> & p_vec, std::vector<double> & d_vec);
  bool getServoGainsByName(const std::string & jn, double & p, double & d);
  bool setServoGains(const std::vector<double> & p_vec, const std::vector<double> & d_vec);
  bool setServoGainsByName(const std::string & jn, double p, double d);

  ControlMode mode() const { return mode_; }
  
private:
  void addLogEntryRobotInfo();
  void setControlMode(const std::string & mode);
  
  RobotConfigParameter config_param_;
  
  std::shared_ptr<RobotControl> robot_;
  
  /*! Global mc_rtc controller */
  mc_control::MCGlobalController & globalController_;
  
  /*! Name of network adaptor */
  std::string network_;
  
  std::chrono::system_clock::time_point now_;
  
  mc_rtc::Logger & logger_;
  double delay_;
  bool controller_init_once_;

  /*! Current control mode, shared across all robot-specific interfaces (H1, G1, Go2, ...) */
  ControlMode mode_ = ControlMode::Position;
};


template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::MCControlUnitree2(mc_control::MCGlobalController & controller, const std::string & network)
  : globalController_(controller), network_(network),  robot_(nullptr),
    //logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc-unitree-"+controller.robot().name()),
    logger_(controller.controller().logger()),
    delay_(0.0), controller_init_once_(false)
{
  // Get parameter values from the configuration file
  auto unitreeConfig = controller.configuration().config("Unitree");
  
  auto robot_name = controller.robot().name();
  if(!unitreeConfig.has(robot_name))
  {
    mc_rtc::log::error(
      "[mc_unitree] A name that matches the controller robot name is not defined in the configuration file");
    return;
  }
  
  mc_rtc::Configuration config_robot = unitreeConfig(robot_name);
  
  /* Get communication information */
  if (network.empty())
  {
    if(!config_robot.has("network-interface"))
    {
      mc_rtc::log::info("[mc_unitree]'network-interface' config entry missing");
      mc_rtc::log::info("[mc_unitree]'run as simulation mode");
      config_param_.network_ = "lo";
    }
    else
    {
      config_robot("network-interface", config_param_.network_);
    }
  }
  else
  {
    config_param_.network_ = network;
  }
  
  auto & robot = controller.controller().robots().robot(robot_name);
  /* Connect to robot (real or simulation) */
  if(!config_param_.network_.empty())
  {
    mc_rtc::log::info("[mc_unitree] Connecting to {} robot on {}",
                      robot_name, config_param_.network_);
  }
  else
  {
    mc_rtc::log::info("[mc_unitree] Running simulation only. No connection to real robot");
  }
  
  if(config_robot.has("kp") && config_robot("kp").size() != config_param_.kp_.size())
  {
    std::vector<double> kp;
    config_robot("kp", kp);

    mc_rtc::log::error("[mc_unitree] Wrong size of kp {} != {}, using default kp",
                        kp.size(), config_param_.kp_.size());
    
    for (size_t i = 0 ; i < kp.size() ; i++)
      config_param_.kp_(i) = kp[i];
  }

  if(config_robot.has("kd") && config_robot("kd").size() != config_param_.kd_.size())
  {
    std::vector<double> kd;
    config_robot("kd", kd);

    mc_rtc::log::error("[mc_unitree] Wrong size of kd {} != {}, using default kd",
                         kd.size(), config_param_.kd_.size());
    
    for (size_t i = 0 ; i < kd.size() ; i++)
      config_param_.kd_(i) = kd[i];
  }
  if(config_robot.has("kp_wait") && config_robot("kp_wait").size() != config_param_.kp_stand_.size())
  {
    std::vector<double> kp;
    config_robot("kp_wait", kp);
    
    mc_rtc::log::error("[mc_unitree] Wrong size of kp_wait {} != {}, using default kp_wait",
                        kp.size(), config_param_.kp_stand_.size());
    
    for (size_t i = 0 ; i < kp.size() ; i++)
      config_param_.kp_stand_(i) = kp[i];
  }
  if(config_robot.has("kd_wait") && config_robot("kd_wait").size() != config_param_.kd_stand_.size())
  {
    std::vector<double> kd;
    config_robot("kd_wait", kd);
    
    mc_rtc::log::error("[mc_unitree] Wrong size of kd_wait {} != {}, using default kd_wait",
                        kd.size(), config_param_.kd_stand_.size());

    for (size_t i = 0 ; i < kd.size() ; i++)
      config_param_.kd_stand_(i) = kd[i];
  }
  if(config_robot.has("mode"))
  {
    mode_ = config_robot("mode");
  }
  else
  {
    mc_rtc::log::info("[mc_unitree] 'mode' config entry missing, using default (Position)");
  }
  
  robot_ = std::make_shared<RobotControl>(this, &robot, config_param_);
  
  // create datastore calls for reading/writing servo pd gains
  controller.controller().datastore().make_call(
    controller.robot().name() + "::GetPDGains",
    [this](std::vector<double> & p, std::vector<double> & d) { return getServoGains(p, d); });
  controller.controller().datastore().make_call(
    controller.robot().name() + "::GetPDGainsByName",
    [this](const std::string & jn, double & p, double & d) { return getServoGainsByName(jn, p, d); });
  controller.controller().datastore().make_call(
    controller.robot().name() + "::SetPDGains",
    [this](const std::vector<double> & p, const std::vector<double> & d) { return setServoGains(p, d); });
  controller.controller().datastore().make_call(
    controller.robot().name() + "::SetPDGainsByName",
    [this](const std::string & jn, double p, double d) { return setServoGainsByName(jn, p, d); });

  mc_rtc::log::info("[mc_unitree] Interface created for robot {}", robot_name);
  
  /* Run QP (every timestep ms) and send result joint commands to the robot */
  now_ = std::chrono::high_resolution_clock::now();
  if(!config_param_.network_.empty())
  {    
    robot_->setInitialState(robot.stance());

    controller.init(robot_->getState().qIn_);
    controller.controller().gui()->addElement(
        {"Robot"}, mc_rtc::gui::Button("Stop controller", [&]() { controller.running = false; }));
    
    addLogEntryRobotInfo();

    controller.running = true;
  }
  
  mc_rtc::log::info("[mc_unitree] interface initialized");

  while (controller.running) {
    sleep(1); 
  }
}

/* Destructor */
template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::~MCControlUnitree2()
{
#if defined(__ENABLE_RT_PREEMPT__)
  pthread_join(lowCmdWriteThread, NULL);
#endif
}

template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
void MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::run(const RobotSensorInfo & state, RobotCommandData & cmdData)
{
  delay_ = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - now_).count();
  now_ = std::chrono::high_resolution_clock::now();
  
  auto start_t_ = std::chrono::high_resolution_clock::now();
  
  /* Send sensor readings to mc_rtc controller */
  globalController_.setSensorOrientation(Eigen::Quaterniond(mc_rbdyn::rpyToMat(state.rpyIn_)));
  globalController_.setSensorAngularVelocity(state.rateIn_);
  globalController_.setSensorLinearAcceleration(state.accIn_);
  globalController_.setEncoderValues(state.qIn_);
  globalController_.setEncoderVelocities(state.dqIn_);
  globalController_.setJointTorques(state.tauIn_);

  if(globalController_.run())
  {
    /* Update control value from the data in a robot */
    auto & robot = globalController_.controller().robots().robot();
    auto jsize = robot.refJointOrder().size();
    auto &datastore = globalController_.controller().datastore();
    if (datastore.has("ControlMode"))
    {
      setControlMode(datastore.get<std::string>("ControlMode"));
    }
    
    for (size_t i = 0 ; i < jsize ; i++)
    {
      auto mcJointId = robot_->refJointOrderToMCJointId(i);
      if (mcJointId == -1)
        continue;

      cmdData.qOut_[i] = robot.mbc().q[mcJointId][0];
      cmdData.dqOut_[i] = robot.mbc().alpha[mcJointId][0];
      cmdData.tauOut_[i] = robot.mbc().jointTorque[mcJointId][0];
      
      // switch(config_param_.mode_)
      // {
      //   case mc_unitree::ControlMode::Position:
      //     cmdData.qOut_[i] = robot.mbc().q[mcJointId][0];
      //     cmdData.dqOut_[i] = robot.mbc().alpha[mcJointId][0];
      //     break;
      //   case mc_unitree::ControlMode::Velocity:
      //     cmdData.dqOut_[i] = robot.mbc().alpha[mcJointId][0];
      //     break;
      //   case mc_unitree::ControlMode::Torque:
      //     cmdData.tauOut_[i] = robot.mbc().jointTorque[mcJointId][0];
      //     break;
      // }
    }
    
    if(config_param_.network_.empty())
    {
      /* Loop back the value of "cmdOut" to "stateIn" */
      robot_->loopbackState(cmdData);
    }
  }

  /* Wait until next controller run */
  if(config_param_.network_.empty())
  {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_t_).count();
    if(elapsed > globalController_.timestep() * 1000)
    {
      mc_rtc::log::warning(
        "[mc_unitree] Loop time {} exeeded timestep of {} ms", elapsed, globalController_.timestep() * 1000);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::microseconds(
          static_cast<unsigned int>((globalController_.timestep() * 1000 - elapsed)) * 1000));
    }
  }
}

template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
void MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::setControlMode(const std::string & mode)
{
  if (mode == "Position") { mode_ = ControlMode::Position; return; }
  if (mode == "Velocity") { mode_ = ControlMode::Velocity; return; }
  if (mode == "Torque")   { mode_ = ControlMode::Torque; return; }
  mc_rtc::log::error("[mc_unitree] {} ControlMode not supported", mode);
}

template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
bool MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::getServoGains(std::vector<double> & p_vec, std::vector<double> & d_vec)
{
  auto jsize = globalController_.controller().robots().robot().refJointOrder().size();
  p_vec.resize(jsize);
  d_vec.resize(jsize);
  for(size_t i = 0; i < jsize; i++)
  {
    p_vec[i] = robot_->kp(i);
    d_vec[i] = robot_->kd(i);
  }
  
  return true;
}

template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
bool MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::getServoGainsByName(const std::string & jn, double & p, double & d)
{
  const auto & rjo = globalController_.controller().robots().robot().refJointOrder();
  auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
  if(rjo_it == rjo.end())
  {
    mc_rtc::log::warning("[mc_unitree2] {}::GetPDGainsByName failed. Joint {} not found in ref_joint_order.",
                         globalController_.controller().robots().robot().name(), jn);
    return false;
  }
  int rjo_idx = std::distance(rjo.begin(), rjo_it);
  
  p = robot_->kp(rjo_idx);
  d = robot_->kd(rjo_idx);
  
  return true;
}

template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
bool MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::setServoGains(const std::vector<double> & p_vec, const std::vector<double> & d_vec)
{
  const auto & rjo = globalController_.controller().robots().robot().refJointOrder();
  if(p_vec.size() != rjo.size())
  {
    mc_rtc::log::critical("[mc_unitree2] {} failed! p_vec is size {} but should be {}.", __func__, p_vec.size(),
                          rjo.size());
    return false;
  }
  if(d_vec.size() != rjo.size())
  {
    mc_rtc::log::critical("[mc_unitree2] {} failed! d_vec is size {} but should be {}.", __func__, d_vec.size(),
                          rjo.size());
    return false;
  }

  for(unsigned int i = 0; i < rjo.size(); i++)
  {
    robot_->setKp(i, p_vec[i]);
    robot_->setKd(i, d_vec[i]);
  }
  
  return true;
}

template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
bool MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::setServoGainsByName(const std::string & jn, double p, double d)
{
  const auto & rjo = globalController_.controller().robots().robot().refJointOrder();
  auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
  if(rjo_it == rjo.end())
  {
    mc_rtc::log::warning("[mc_unitree2] {}::SetPDGainsByName failed. Joint {} not found in ref_joint_order.",
                         globalController_.controller().robots().robot().name(), jn);
    return false;
  }
  int rjo_idx = std::distance(rjo.begin(), rjo_it);
  robot_->setKp(robot_->mcJointIdToJointId(rjo_idx), p);
  robot_->setKd(robot_->mcJointIdToJointId(rjo_idx), d);
  
  return true;
}
  
template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
 void MCControlUnitree2<RobotControl, RobotSensorInfo, RobotCommandData, RobotConfigParameter>::addLogEntryRobotInfo()
{
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  + "::time", this, [&]() { return robot_->time(); });
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  + "::time_run", this, [&]() { return robot_->timeRun(); });
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name() + "::delay", this, [&]() { return delay_; });

  /* Sensors */
  /* Position(Angle) values */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  +"::measured_joint_position", [this]() { return robot_->getState().qIn_; });
  /* Velocity values */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  +"::measured_joint_velocity", [this]() { return robot_->getState().dqIn_; });
  /* Torque values */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  +"::measured_joint_torque", [this](){ return robot_->getState().tauIn_; });
  /* Orientation sensor */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  +"::measured_imu_rpy", [this]() { return robot_->getState().rpyIn_; });
  /* Accelerometer */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  +"::measured_imu_accel", [this]() { return robot_->getState().accIn_; });
  /* Angular velocity */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  + "::measured_imu_rate", [this]() { return robot_->getState().rateIn_; });
  
  /* Command data to send to the robot */
  /* Position(Angle) values */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  + "::command_joint_position", [this]() { return robot_->getCommand().qOut_; });
  /* Velocity values */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  + "::command_joint_velocity", [this]() { return robot_->getCommand().dqOut_; });
  /* Torque values */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  + "::command_joint_torque", [this]() { return robot_->getCommand().tauOut_; });

  /* PD gains */
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  + "::command_joint_kp", [this]() { return robot_->getCommand().kpOut_; });
  globalController_.controller().logger().addLogEntry(globalController_.controller().robot().name()  + "::command_joint_kd", [this]() { return robot_->getCommand().kdOut_; });

  mc_rtc::log::success("Entries added to logger");
}
  
} // namespace mc_unitree
