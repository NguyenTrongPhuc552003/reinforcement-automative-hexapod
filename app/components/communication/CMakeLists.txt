idf_component_register(SRCS "ble_comm.cpp"
                       SRCS "wifi_comm.cpp"
                      INCLUDE_DIRS "."
                      REQUIRES
                        esp_wifi
                        nvs_flash
                        esp_netif
                        driver)

# Include the component's header files`
target_include_directories(${COMPONENT_LIB} PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/include"
)
