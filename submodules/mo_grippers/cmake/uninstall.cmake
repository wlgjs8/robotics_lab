if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "CMake run directly by the user")
    # Add the uninstall script
    if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt")
        add_custom_target(uninstall
            COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
        )
        # Generate the uninstall script
        configure_file(
            ${CMAKE_CURRENT_SOURCE_DIR}/cmake/in/uninstall.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
            @ONLY
        )
    else()
        message(WARNING "Install manifest not found. Install the project before uninstalling.")
    endif()

else()
	message(STATUS "CMake invoked as part of another project (e.g., via FetchContent)")
endif()