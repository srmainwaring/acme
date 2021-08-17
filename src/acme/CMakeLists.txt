add_library(acme)

target_include_directories(acme PUBLIC ${CMAKE_CURRENT_LIST_DIR}/..)

target_sources(acme PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/version.cpp")

add_dependencies(acme check_git_${PROJECT_NAME}) # For git_watcher to fetch git informations before effective build

target_link_libraries(acme
        MathUtils::MathUtils
        nlohmann_json)

add_subdirectory(propeller)
add_subdirectory(rudder)
add_subdirectory(tunnel)
add_subdirectory(propeller_rudder)
add_subdirectory(sail)

set_target_properties(acme PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")