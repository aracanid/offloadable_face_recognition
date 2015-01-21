# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "offloadable_face_recognition: 2 messages, 2 services")

set(MSG_I_FLAGS "-Ioffloadable_face_recognition:/home/justin/catkin_ws/src/offloadable_face_recognition/msg;-Igeometry_msgs:/home/justin/catkin_ws/src/common_msgs/geometry_msgs/msg;-Istd_msgs:/home/justin/catkin_ws/src/std_msgs/msg;-Isensor_msgs:/home/justin/catkin_ws/src/common_msgs/sensor_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(offloadable_face_recognition_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg" NAME_WE)
add_custom_target(_offloadable_face_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "offloadable_face_recognition" "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg" ""
)

get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg" NAME_WE)
add_custom_target(_offloadable_face_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "offloadable_face_recognition" "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg" ""
)

get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/AddFeatures.srv" NAME_WE)
add_custom_target(_offloadable_face_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "offloadable_face_recognition" "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/AddFeatures.srv" "offloadable_face_recognition/FaceBox:offloadable_face_recognition/FeatureCoordinates:sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/PruneFeatures.srv" NAME_WE)
add_custom_target(_offloadable_face_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "offloadable_face_recognition" "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/PruneFeatures.srv" "offloadable_face_recognition/FeatureCoordinates"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offloadable_face_recognition
)
_generate_msg_cpp(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offloadable_face_recognition
)

### Generating Services
_generate_srv_cpp(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/PruneFeatures.srv"
  "${MSG_I_FLAGS}"
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offloadable_face_recognition
)
_generate_srv_cpp(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/AddFeatures.srv"
  "${MSG_I_FLAGS}"
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg;/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg;/home/justin/catkin_ws/src/common_msgs/sensor_msgs/msg/Image.msg;/home/justin/catkin_ws/src/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offloadable_face_recognition
)

### Generating Module File
_generate_module_cpp(offloadable_face_recognition
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offloadable_face_recognition
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(offloadable_face_recognition_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(offloadable_face_recognition_generate_messages offloadable_face_recognition_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_cpp _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_cpp _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/AddFeatures.srv" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_cpp _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/PruneFeatures.srv" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_cpp _offloadable_face_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offloadable_face_recognition_gencpp)
add_dependencies(offloadable_face_recognition_gencpp offloadable_face_recognition_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offloadable_face_recognition_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offloadable_face_recognition
)
_generate_msg_lisp(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offloadable_face_recognition
)

### Generating Services
_generate_srv_lisp(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/PruneFeatures.srv"
  "${MSG_I_FLAGS}"
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offloadable_face_recognition
)
_generate_srv_lisp(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/AddFeatures.srv"
  "${MSG_I_FLAGS}"
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg;/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg;/home/justin/catkin_ws/src/common_msgs/sensor_msgs/msg/Image.msg;/home/justin/catkin_ws/src/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offloadable_face_recognition
)

### Generating Module File
_generate_module_lisp(offloadable_face_recognition
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offloadable_face_recognition
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(offloadable_face_recognition_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(offloadable_face_recognition_generate_messages offloadable_face_recognition_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_lisp _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_lisp _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/AddFeatures.srv" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_lisp _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/PruneFeatures.srv" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_lisp _offloadable_face_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offloadable_face_recognition_genlisp)
add_dependencies(offloadable_face_recognition_genlisp offloadable_face_recognition_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offloadable_face_recognition_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offloadable_face_recognition
)
_generate_msg_py(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offloadable_face_recognition
)

### Generating Services
_generate_srv_py(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/PruneFeatures.srv"
  "${MSG_I_FLAGS}"
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offloadable_face_recognition
)
_generate_srv_py(offloadable_face_recognition
  "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/AddFeatures.srv"
  "${MSG_I_FLAGS}"
  "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg;/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg;/home/justin/catkin_ws/src/common_msgs/sensor_msgs/msg/Image.msg;/home/justin/catkin_ws/src/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offloadable_face_recognition
)

### Generating Module File
_generate_module_py(offloadable_face_recognition
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offloadable_face_recognition
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(offloadable_face_recognition_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(offloadable_face_recognition_generate_messages offloadable_face_recognition_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FaceBox.msg" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_py _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/msg/FeatureCoordinates.msg" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_py _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/AddFeatures.srv" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_py _offloadable_face_recognition_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/catkin_ws/src/offloadable_face_recognition/srv/PruneFeatures.srv" NAME_WE)
add_dependencies(offloadable_face_recognition_generate_messages_py _offloadable_face_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offloadable_face_recognition_genpy)
add_dependencies(offloadable_face_recognition_genpy offloadable_face_recognition_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offloadable_face_recognition_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offloadable_face_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offloadable_face_recognition
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(offloadable_face_recognition_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(offloadable_face_recognition_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(offloadable_face_recognition_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offloadable_face_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offloadable_face_recognition
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(offloadable_face_recognition_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(offloadable_face_recognition_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(offloadable_face_recognition_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offloadable_face_recognition)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offloadable_face_recognition\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offloadable_face_recognition
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(offloadable_face_recognition_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(offloadable_face_recognition_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(offloadable_face_recognition_generate_messages_py sensor_msgs_generate_messages_py)
