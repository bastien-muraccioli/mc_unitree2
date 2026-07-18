option(GENERATE_GO2_CONTROLLER "Build Go2 controller" OFF)
option(GENERATE_H1_CONTROLLER "Build H1 controller" OFF)

option(ENABLE_RT_PREEMPT "Create thread on RT preemt kernel" OFF)
if (ENABLE_RT_PREEMPT)
  add_definitions(-D__ENABLE_RT_PREEMPT__)
endif()

set(UNITREE_SDK2_SRC_DIR unitree_sdk2 CACHE STRING "Set unitree sdk2 source directory")

set(CMAKE_INSTALL_RPATH "$ENV{LD_LIBRARY_PATH}")

include_directories(
  ${CMAKE_INSTALL_PREFIX}/include/ddscxx
)
link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_SYSTEM_PROCESSOR}
)

set(HEADERS
  ControlMode.h
  MCControlUnitree2.h
)

set(SOURCES
  rtapi.cpp
)

if (GENERATE_GO2_CONTROLLER)
  
  set(GO2_CONTROLLER_NAME MCControlGo2)
  set(GO2_HEADERS
    Go2Control.h
  )
  
  set(GO2_SOURCES
    main_go2.cpp
    Go2Control.cpp
    ${UNITREE_SDK2_SRC_DIR}/example/go2/low_level/motors.cpp
  )

  message(STATUS "unitree_sdk2 directory set to: ${UNITREE_SDK2_SRC_DIR}")
  include_directories(
    ${UNITREE_SDK2_SRC_DIR}/example
    ${CMAKE_CURRENT_SOURCE_DIR}/../libfort
  )
  
  add_executable(${GO2_CONTROLLER_NAME}
    ${SOURCES}
    ${GO2_SOURCES}
    CLI11::CLI11
  )
  
  target_link_libraries(${GO2_CONTROLLER_NAME}
    PUBLIC
    pthread
    mc_rtc::mc_control
    Boost::program_options
    unitree_sdk2
    fort
  )
  
  install(TARGETS ${GO2_CONTROLLER_NAME} DESTINATION bin)
  
endif()

if (GENERATE_H1_CONTROLLER)
  
  set(H1_CONTROLLER_NAME MCControlH1)
  set(H1_HEADERS
    H1Control.h
  )
  set(H1_SOURCES
    main_h1.cpp
    H1Control.cpp
    ${UNITREE_SDK2_SRC_DIR}/example/h1/low_level/motors.cpp
  )
  
  message(STATUS "unitree_sdk2 directory set to: ${UNITREE_SDK2_SRC_DIR}")
  include_directories(
    ${UNITREE_SDK2_SRC_DIR}/example
    ${CMAKE_CURRENT_SOURCE_DIR}/../libfort
  )
  
  add_executable(${H1_CONTROLLER_NAME}
    ${SOURCES}
    ${H1_SOURCES}
  )
  
  target_link_libraries(${H1_CONTROLLER_NAME}
    PUBLIC
    pthread
    mc_rtc::mc_control
    Boost::program_options
    unitree_sdk2
    fort
    CLI11::CLI11
  )
  
  install(TARGETS ${H1_CONTROLLER_NAME} DESTINATION bin)
  
endif()



