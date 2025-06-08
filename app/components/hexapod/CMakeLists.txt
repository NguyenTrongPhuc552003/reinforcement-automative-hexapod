idf_component_register(
    SRCS "hexapod.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        leg
        kinematics
        servo_controller
)
