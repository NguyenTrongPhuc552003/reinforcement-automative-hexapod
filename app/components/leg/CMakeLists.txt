idf_component_register(
    SRCS "leg.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        servo_controller
        kinematics
)
