# Examples CMakeLists.txt

add_executable(td3_simple td3_simple.cpp)
target_link_libraries(td3_simple PRIVATE td3learn)

install(TARGETS td3_simple
    RUNTIME DESTINATION bin
)
