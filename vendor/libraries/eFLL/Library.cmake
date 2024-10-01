file(GLOB DUNE_EFLL_FILES
  user/vendor/libraries/eFLL/*.cpp
  user/vendor/libraries/eFLL/*.h
 )

set_source_files_properties(${DUNE_EFLL_FILES}
  PROPERTIES COMPILE_FLAGS "${DUNE_CXX_FLAGS} ${DUNE_CXX_FLAGS_STRICT}")
  
list(APPEND DUNE_VENDOR_FILES ${DUNE_EFLL_FILES})

set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries)