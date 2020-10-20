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

####### setting g2o libraries #######################################

SET (G2O_LIBS csparse cxsparse g2o_csparse_extension g2o_cli g2o_core g2o_interface g2o_parser g2o_solver_cholmod
g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp
g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)

SET(CUSTOM_G2O_TYPES_SOURCES 
              camera_projection.cpp
              vertex_landmarkxyz.cpp
              vertex_pose.cpp
              edge_pose_landmark_reproject.cpp
              edge_pose_pose.cpp
              register_types.cpp)

ADD_LIBRARY(g2o_custom_types ${CUSTOM_G2O_TYPES_SOURCES})
target_link_libraries(g2o_custom_types Eigen3::Eigen GL GLU glut ${G2O_LIBS})
