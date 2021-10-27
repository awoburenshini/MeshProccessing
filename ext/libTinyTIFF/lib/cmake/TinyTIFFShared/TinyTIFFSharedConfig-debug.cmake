#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "TinyTIFFShared" for configuration "Debug"
set_property(TARGET TinyTIFFShared APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(TinyTIFFShared PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/TinyTIFFShared_Debug.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/TinyTIFFShared_Debug.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS TinyTIFFShared )
list(APPEND _IMPORT_CHECK_FILES_FOR_TinyTIFFShared "${_IMPORT_PREFIX}/lib/TinyTIFFShared_Debug.lib" "${_IMPORT_PREFIX}/bin/TinyTIFFShared_Debug.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
