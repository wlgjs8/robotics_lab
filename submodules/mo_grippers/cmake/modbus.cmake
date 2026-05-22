find_package(PkgConfig QUIET REQUIRED)
pkg_check_modules(libmodbus REQUIRED libmodbus)

if(NOT libmodbus_FOUND)
  message(FATAL_ERROR "libmodbus does not exist")
else()
  include_directories("/usr/include/modbus" "/usr/local/include/modbus")
  if(libmodbus_VERSION VERSION_EQUAL ${MODBUS_REQUIRED_VERSION})
    message(STATUS "libmodbus version is ${MODBUS_REQUIRED_VERSION}, which is the expected version")
  elseif(libmodbus_VERSION VERSION_LESS ${MODBUS_REQUIRED_VERSION})
    message(
      FATAL_ERROR
        "libmodbus version ${libmodbus_VERSION} is too old. Version ${MODBUS_REQUIRED_VERSION} is required."
    )
  else()
    message(
      WARNING
        "libmodbus version ${libmodbus_VERSION} is newer than the expected version ${MODBUS_REQUIRED_VERSION}"
    )
  endif()
endif()
