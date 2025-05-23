cmake_minimum_required(VERSION 2.8.9)
project(slam_app)

set(LIB_CXSPARSE  /usr/lib/x86_64-linux-gnu/libcxsparse.so)

set(G2O_LIBS
  ${G2O_CLI_LIBRARY}
  ${G2O_CLI_LIBRARY_DEBUG}
  ${G2O_CORE_LIBRARY}
  ${G2O_CORE_LIBRARY_DEBUG}
  ${G2O_INCLUDE_DIR}
  ${G2O_SIMULATOR}
  ${G2O_SIMULATOR_DEBUG}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CHOLMOD_DEBUG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_DEBUG}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE_EXTENSION_DEBUG}
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_DENSE_DEBUG}
  ${G2O_SOLVER_EIGEN}
  ${G2O_SOLVER_EIGEN_DEBUG}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_PCG_DEBUG}
  ${G2O_SOLVER_SLAM2D_LINEAR}
  ${G2O_SOLVER_SLAM2D_LINEAR_DEBUG}
  ${G2O_SOLVER_STRUCTURE_ONLY}
  ${G2O_SOLVER_STRUCTURE_ONLY_DEBUG}
  ${G2O_STUFF_LIBRARY}
  ${G2O_STUFF_LIBRARY_DEBUG}
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_DATA_DEBUG}
  ${G2O_TYPES_ICP}
  ${G2O_TYPES_ICP_DEBUG}
  ${G2O_TYPES_SBA}
  ${G2O_TYPES_SBA_DEBUG}
  ${G2O_TYPES_SCLAM2D}
  ${G2O_TYPES_SCLAM2D_DEBUG}
  ${G2O_TYPES_SIM3}
  ${G2O_TYPES_SIM3_DEBUG}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM2D_DEBUG}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_DEBUG}
)

## demos
message("LsdSlam_ALL_LIBRARIES=${LsdSlam_ALL_LIBRARIES}")
add_executable(main_on_images main_on_images.cc)
add_executable(main_on_images_sock main_on_images_sock.cc)
set_property(TARGET main_on_images PROPERTY FOLDER "lsd_slam/apps")
set_property(TARGET main_on_images_sock PROPERTY FOLDER "lsd_slam/apps")

target_link_libraries(main_on_images ${LsdSlam_ALL_LIBRARIES} ${LIB_CXSPARSE} ${OpenCV_LIBRARIES})
target_link_libraries(main_on_images_sock ${LsdSlam_ALL_LIBRARIES} ${LIB_CXSPARSE} ${OpenCV_LIBRARIES})

## only this works at the moment
add_executable(sample_app sample_app.cc DebugOutput3DWrapper.cpp DebugOutput3DWrapper.h)
set_property(TARGET sample_app PROPERTY FOLDER "lsd_slam/apps")
target_link_libraries(sample_app ${LsdSlam_ALL_LIBRARIES} ${G2O_LIBS} ${LIB_CXSPARSE})

lsd_slam_print_status("LsdSlam_ALL_LIBRARIES:${LsdSlam_ALL_LIBRARIES}")
