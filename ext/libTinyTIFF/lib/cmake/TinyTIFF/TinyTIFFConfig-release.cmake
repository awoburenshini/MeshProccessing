#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "TinyTIFF" for configuration "Release"
set_property(TARGET TinyTIFF APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(TinyTIFF PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/TinyTIFF_Release.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS TinyTIFF )
list(APPEND _IMPORT_CHECK_FILES_FOR_TinyTIFF "${_IMPORT_PREFIX}/lib/TinyTIFF_Release.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
