#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "TinyTIFFShared" for configuration "Release"
set_property(TARGET TinyTIFFShared APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(TinyTIFFShared PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/TinyTIFFShared_Release.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/TinyTIFFShared_Release.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS TinyTIFFShared )
list(APPEND _IMPORT_CHECK_FILES_FOR_TinyTIFFShared "${_IMPORT_PREFIX}/lib/TinyTIFFShared_Release.lib" "${_IMPORT_PREFIX}/bin/TinyTIFFShared_Release.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
