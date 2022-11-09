
# Here we declare the different PATH, TAG and PATCH to get the ACME dependencies

# JSON
set(json_URL https://github.com/nlohmann/json.git)
set(json_TAG v3.6.1 CACHE STRING "json version")

# MathUtils
# set(mathutils_URL git@frydom-ce.org:ce/mathutils.git)
# set(mathutils_TAG v1.8 CACHE STRING "mathutils version")
set(mathutils_URL https://github.com/srmainwaring/mathutils.git)
set(mathutils_TAG c1b5cfb4a992b82c2721d9dc7f5d8a6e2817d14e CACHE STRING "MathUtils version")
set(MATHUTILS_BUILD_TESTS OFF CACHE BOOL "")
set(MATHUTILS_BUILD_BOOST_TESTS OFF CACHE BOOL "")

# GoogleTest
set(googletest_URL https://github.com/google/googletest.git)
set(googletest_TAG release-1.10.0 CACHE STRING "googletest version")

# hermes
# set(hermes_URL git@frydom-ce.org:ce/hermes.git)
# set(hermes_TAG v1.4 CACHE STRING " version")
set(hermes_URL https://github.com/srmainwaring/hermes.git)
set(hermes_TAG 5e27fdab4376f7cefc27a2066e4b82bf6d928e6c CACHE STRING " version")
