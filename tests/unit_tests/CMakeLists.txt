
set(UNIT_TESTS
        test_acme_FPP1Q
        test_acme_FPP4Q
        test_acme_CPP
        test_acme_simpleRudder
        test_acme_FlapRudder
        test_acme_BrixRudder
        )

foreach (test ${UNIT_TESTS})
    message(STATUS "        ...add ${test}")

    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} acme gtest_main)

    add_test(NAME ${test}
            COMMAND ${test})
endforeach ()
