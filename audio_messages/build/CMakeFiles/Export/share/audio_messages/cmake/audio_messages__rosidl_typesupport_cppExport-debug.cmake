#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "audio_messages::audio_messages__rosidl_typesupport_cpp" for configuration "Debug"
set_property(TARGET audio_messages::audio_messages__rosidl_typesupport_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(audio_messages::audio_messages__rosidl_typesupport_cpp PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_cpp::rosidl_typesupport_cpp;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libaudio_messages__rosidl_typesupport_cpp.so"
  IMPORTED_SONAME_DEBUG "libaudio_messages__rosidl_typesupport_cpp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS audio_messages::audio_messages__rosidl_typesupport_cpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_audio_messages::audio_messages__rosidl_typesupport_cpp "${_IMPORT_PREFIX}/lib/libaudio_messages__rosidl_typesupport_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
