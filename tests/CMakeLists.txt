message(STATUS "    ...tests")

message(STATUS "Test programs for ACME framework...")

if (${ACME_BUILD_DEV_TESTS})
    add_subdirectory(dev_tests)
endif ()

if (${ACME_BUILD_UNIT_TESTS})
    add_subdirectory(unit_tests)
endif ()

