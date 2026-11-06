cmake_minimum_required(VERSION 3.16)
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" GUANID_VERSION LIMIT_COUNT 1)
project(guanid VERSION ${GUANID_VERSION} LANGUAGES CXX)

add_library(guanid INTERFACE)
add_library(guanid::guanid ALIAS guanid)
target_include_directories(guanid INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(guanid INTERFACE cxx_std_17)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/guanidConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/guanidConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guanid")
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/guanidConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMinorVersion)
install(TARGETS guanid EXPORT guanidTargets)
install(DIRECTORY include/guanid DESTINATION include)
install(EXPORT guanidTargets NAMESPACE guanid:: DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guanid")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/guanidConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/guanidConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/guanid")
install(FILES README.md LICENSE NOTICE CHANGELOG.md VERSION CONTRIBUTING.md SECURITY.md
    DESTINATION "${CMAKE_INSTALL_DATADIR}/guanid")
install(DIRECTORY docs examples DESTINATION "${CMAKE_INSTALL_DATADIR}/guanid")
install(FILES tools/analyze.cpp DESTINATION "${CMAKE_INSTALL_DATADIR}/guanid/tools")

option(GUANID_BUILD_ANALYZER "Build the dependency-free C++ analyzer" ON)
if(GUANID_BUILD_ANALYZER)
    add_executable(guanid_analyze tools/analyze.cpp)
    target_link_libraries(guanid_analyze PRIVATE guanid)
    install(TARGETS guanid_analyze RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

option(GUANID_BUILD_EXAMPLES "Build host-only capture examples" OFF)
if(GUANID_BUILD_EXAMPLES)
    add_executable(guanid_simulated_capture examples/simulated_capture.cpp)
    target_link_libraries(guanid_simulated_capture PRIVATE guanid)
    add_executable(guanid_callback_capture examples/callback_capture.cpp)
    target_link_libraries(guanid_callback_capture PRIVATE guanid)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(guanid_tests tests/core.cpp)
    target_link_libraries(guanid_tests PRIVATE guanid)
    add_test(NAME guanid_core COMMAND guanid_tests)
    add_executable(guanid_routine_tests tests/routine.cpp)
    target_link_libraries(guanid_routine_tests PRIVATE guanid)
    add_test(NAME guanid_routine COMMAND guanid_routine_tests)
    add_executable(guanid_sysid_tests tests/sysid.cpp)
    target_link_libraries(guanid_sysid_tests PRIVATE guanid)
    add_test(NAME guanid_sysid COMMAND guanid_sysid_tests)
    add_executable(guanid_analysis_tests tests/analysis.cpp)
    target_link_libraries(guanid_analysis_tests PRIVATE guanid)
    add_test(NAME guanid_analysis COMMAND guanid_analysis_tests)
    add_executable(guanid_track_width_tests tests/track_width.cpp)
    target_link_libraries(guanid_track_width_tests PRIVATE guanid)
    add_test(NAME guanid_track_width COMMAND guanid_track_width_tests)
    add_executable(guanid_pros_tests tests/pros.cpp)
    target_include_directories(guanid_pros_tests PRIVATE tests/stubs)
    target_link_libraries(guanid_pros_tests PRIVATE guanid)
    add_test(NAME guanid_pros COMMAND guanid_pros_tests)
    add_executable(guanid_drivetrain_tests tests/drivetrain.cpp)
    target_include_directories(guanid_drivetrain_tests PRIVATE tests/stubs)
    target_link_libraries(guanid_drivetrain_tests PRIVATE guanid)
    add_test(NAME guanid_drivetrain COMMAND guanid_drivetrain_tests)
    if(GUANID_BUILD_EXAMPLES AND GUANID_BUILD_ANALYZER)
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(NAME guanid_public_units COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/public_units.py $<TARGET_FILE_DIR:guanid_analyze>)
        endif()
        add_test(NAME guanid_cli COMMAND ${CMAKE_COMMAND}
            -DCAPTURE=$<TARGET_FILE:guanid_callback_capture>
            -DANALYZE=$<TARGET_FILE:guanid_analyze>
            -DWORK=${CMAKE_CURRENT_BINARY_DIR}/cli-test
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cli.cmake)
    endif()
endif()
