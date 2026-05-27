find_package(plogger QUIET)

if(NOT plogger_FOUND)
  message(STATUS "plogger does not exist")
  include(FetchContent)
  fetchcontent_declare(
    plogger
    GIT_REPOSITORY https://github.com/PLAIF-dev/sw_logger.git
    GIT_TAG v1.2.0
  )

  fetchcontent_makeavailable(plogger)
else()
  message(STATUS "plogger found: ${plogger_VERSION}")
endif()
