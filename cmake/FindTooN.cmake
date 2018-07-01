
find_path(TOON_INCLUDE_DIR NAMES TooN/TooN.h
  HINTS
  ENV TOON_ROOT
  ENV TOON_ROOT_DIR
  PATHS
	~/usr/include
	~/usr/.local/include
	~/.local/include
	~/usr/local/include
	/usr/include
	/usr/local/include
  # NO_DEFAULT_PATH
)

if(TOON_INCLUDE_DIR)
	set(TooN_FOUND TRUE)
	set(TOON_INCLUDE_DIRS ${TOON_INCLUDE_DIR} CACHE STRING "The include paths needed to use TooN")
endif()

mark_as_advanced(
	TOON_INCLUDE_DIRS
)


# Generate appropriate messages
if(TooN_FOUND)
    if(NOT TooN_FIND_QUIETLY)
    	   message("-- Found Toon: ${TOON_INCLUDE_DIR}")
    endif(NOT TooN_FIND_QUIETLY)
else(TooN_FOUND)
    if(TooN_FIND_REQUIRED)
	message(FATAL_ERROR "-- Could NOT find TooN (missing: TOON_INCLUDE_DIR)")
    endif(TooN_FIND_REQUIRED)
endif(TooN_FOUND)
