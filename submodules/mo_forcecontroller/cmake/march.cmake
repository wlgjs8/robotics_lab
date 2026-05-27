find_package(plogger QUIET)
find_package(march QUIET)

if(NOT march_FOUND)
  message(STATUS "march does not exist")
  include(FetchContent)
  fetchcontent_declare(
    march
    GIT_REPOSITORY https://github.com/PLAIF-dev/sw_march.git
    GIT_TAG v1.1.2
  )

  fetchcontent_makeavailable(march)
else()
  message(STATUS "march found: ${march_VERSION}")
endif()
