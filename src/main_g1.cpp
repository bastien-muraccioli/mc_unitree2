#include "G1Control.h"

#include <mc_rtc/config.h>

#include <fstream>
#include <filesystem>
#include <CLI/CLI.hpp>
#include <cstdio>
#include <unistd.h>

namespace fs = std::filesystem;

namespace
{

/* Checking the existence of the file */
/* Return value: true if the file exists, false otherwise */
bool file_exists(const std::string& str)
{
  std::ifstream fs(str);
  return fs.is_open();
}

} // namespace

/* Main function of the interface */
int main(int argc, char * argv[])
{
  /* Set command line arguments options */
  /* Usage example: MCControlUnitree2 -n lo -f @ETC_PATH@/mc_unitree/mc_rtc_xxxxx.yaml */
  
  fs::path config_path = mc_rtc::user_config_directory_path("mc_rtc.conf");
  // Load user's local configuration if it exists
  if(!fs::exists(config_path)) { config_path.replace_extension(".yaml"); }
  std::string check_file = config_path.string();
  if(!file_exists(check_file))
  {
    check_file = "";
  }
  
  std::string conf_file = check_file;
  std::string network;
  
  CLI::App app{"MCControlG1 options"};
  app.add_option("-n,--network", network, "Name of network adaptor")->default_val("");
  app.add_option("-f,--conf", conf_file, "Configuration file")->default_val(check_file);
  bool show_sensors = false;
  double sensors_period = 0.5;
  app.add_flag("-s,--show-sensors", show_sensors,
               "Show a live dashboard of the sensors (IMUs, joints, hands) instead of the status messages");
  app.add_option("--sensors-period", sensors_period, "Refresh period of the sensor dashboard [s]")->default_val(0.5);
  bool no_color = false;
  app.add_flag("--no-color", no_color, "Sensor dashboard without colors");
  
  /* Parse command line arguments */
  try
  {
    app.parse(argc, argv);
  }
  catch(const CLI::ParseError& e)
  {
    return app.exit(e);
  }
  mc_rtc::log::info("[mc_unitree] Reading additional configuration from {}", conf_file);
  // Colors only when writing to a terminal (not when the output is redirected to a file)
  mc_unitree::G1Control::ShowSensors(show_sensors, sensors_period, !no_color && isatty(fileno(stdout)));

  /* Create global controller */
  mc_control::MCGlobalController g_controller(conf_file, nullptr);

  /* Check that the interface can work with the main controller robot */
  /* mc_g1 robot names are g1_23dof or g1_29dof (MainRobot: G1, G1_23dof or G1_29dof) */
  const std::string & robot_name = g_controller.robot().name();
  if(robot_name.rfind(mc_unitree::ROBOT_NAME, 0) != 0)
  {
    mc_rtc::log::error("[mc_unitree] This program can only handle '{}' robots (got '{}')", mc_unitree::ROBOT_NAME,
                       robot_name);
    return 1;
  }
  
  /* Create MCControlUnitree2 interface */
  mc_unitree::MCControlUnitree2<mc_unitree::G1Control,
                                mc_unitree::G1SensorInfo,
                                mc_unitree::G1CommandData,
                                mc_unitree::G1ConfigParameter> mc_control_unitree(g_controller,
                                                                                  network);
  
  mc_rtc::log::info("[mc_unitree] Terminated");
  
  return 0;
}
