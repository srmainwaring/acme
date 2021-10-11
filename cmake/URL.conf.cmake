
# Here we declare the different PATH, TAG and PATCH to get the ACME dependencies

# JSON
set(json_URL https://github.com/nlohmann/json.git)
set(json_TAG v3.6.1 CACHE STRING "json version")

# MathUtils
set(mathutils_URL git@frydom-ce.org:ce/mathutils.git)
set(mathutils_TAG v1.8 CACHE STRING "mathutils version")
set(MATHUTILS_BUILD_TESTS OFF CACHE BOOL "")
set(MATHUTILS_BUILD_BOOST_TESTS OFF CACHE BOOL "")

# GoogleTest
set(googletest_URL https://github.com/google/googletest.git)
set(googletest_TAG release-1.10.0 CACHE STRING "googletest version")

# hermes
set(hermes_URL git@frydom-ce.org:ce/hermes.git)
set(hermes_TAG v1.4 CACHE STRING " version")
