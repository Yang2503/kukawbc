cmake_minimum_required(VERSION 3.10)
project(kuka-control)

IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
ENDIF ()
add_compile_options(-std=c++17)

#系统eigen版本可能老旧，使用自供eigen
set(dirEigen "third_party/eigen3")
set(dirPino "third_party/pinocchio")
set(dirJson "third_party/jsoncpp")

set(dirQP "third_party/qpOASES")
set(libURDF "third_party/urdfdom")

set(incBoost "third_party/boost")
# 设置 Webots 头文件路径
set(WEBOTS_INCLUDE_DIR "/usr/local/webots/include/controller/cpp")
# 设置 Webots 库文件路径
set(WEBOTS_LIBRARY_DIR "/usr/local/webots/lib/controller")


set(allDir ${dirEigen} ${dirPino} ${dirJson} ${dirQP} )
set(allLib ${allDir} ${libURDF} ${WEBOTS_LIBRARY_DIR})
set(allInc ${allDir} ${incBoost} ${WEBOTS_INCLUDE_DIR})

include_directories(${allInc})
include_directories("algotithm")
include_directories("common")
include_directories("math")
include_directories("sim_interface")

message(${CMAKE_SYSTEM_PROCESSOR})
link_directories(${allLib})

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp algotithm/*.cpp common/*.cpp math/*.cpp sim_interface/*.cpp)
file(GLOB HEADER_FILES *.h algotithm/*.h common/*.h math/*.h sim_interface/*.h)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES} ${HEADER_FILES})

#链接库
set(corLibs pinocchio urdfdom_model  tinyxml console_bridge jsoncpp qpOASES)# Controller CppController)
# 修改可执行文件链接的库，移除 mujoco 相关库，添加 Webots 相关库
set(WEBOTS_LIBS Controller CppController)
#根据架构，处理连接库
set(sysCoreLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
	message("linux arm64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_arm64)
	endforeach()
	list(APPEND sysCoreLibs dl)#arm 需要 -ldl
else()
	message("linux x64架构")
	foreach(lib ${corLibs})
		list(APPEND sysCoreLibs ${lib}_lin_x64)
	endforeach()
endif()
message(${sysCoreLibs})

#生成控制核心库
add_library(core ${SOURCES})
target_link_libraries(core ${sysCoreLibs} pthread)

#生成仿真可执行文件
add_executable(ball_controller controllers/ball_controller/ball_controller.cpp)
set_target_properties(ball_controller PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/controllers/ball_controller)
target_link_libraries(ball_controller core ${WEBOTS_LIBS} dl)
add_executable(weightwbc_v controllers/weightwbc_v/weightwbc_v.cpp)
set_target_properties(weightwbc_v PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/controllers/weightwbc_v)
target_link_libraries(weightwbc_v core ${WEBOTS_LIBS} dl)


