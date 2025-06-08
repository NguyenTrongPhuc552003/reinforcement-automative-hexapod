idf_component_register(
    SRCS "gait_controller.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        kinematics
        hexapod
)
