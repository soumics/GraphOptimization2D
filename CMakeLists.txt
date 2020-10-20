cmake_minimum_required (VERSION 3.0.0)


####### project directory setup ################################### 
set(PROJECT_NAME GraphSlam)
project (${PROJECT_NAME})

####### output directory path #####################################
set(EXECUTABLE_OUTPUT_PATH ./${CMAKE_BUILD_TYPE})

####### C++ VERSION Flags #########################################
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

####### finding required packages for this project ################
find_package(Eigen3 3.3 REQUIRED)
if (NOT Eigen3_FOUND)
  MESSAGE ("Eigen3 NOT FOUND")
ENDIF()

find_package(g2o REQUIRED)
if (NOT g2o_FOUND)
  MESSAGE ("g2o NOT FOUND")
ENDIF()


######## including the required package directories ################
include_directories(${EIGEN_INCLUDE_DIR} ${g2o_INCLUDE_DIR})

######## setting the SOURCE directory ##############################
set (SOURCE src/main.cpp)

####### setting g2o libraries #######################################

SET (G2O_LIBS csparse cxsparse g2o_csparse_extension g2o_cli g2o_core g2o_interface g2o_parser g2o_solver_cholmod
g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp
g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)

####### adding g2o custom types #####################################
#set(G2O_CUSTOM_TYPES src/custom_types)
#ADD_SUBDIRECTORY(${G2O_CUSTOM_TYPES})
ADD_SUBDIRECTORY(src/custom_types)
 
add_executable (${PROJECT_NAME} ${SOURCE})
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen ${G2O_LIBS} g2o_custom_types)





