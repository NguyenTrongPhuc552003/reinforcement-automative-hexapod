# Examples CMakeLists.txt

add_executable(demo demo.cpp)
target_link_libraries(demo PRIVATE td3learn)

install(TARGETS demo
    RUNTIME DESTINATION bin
)
