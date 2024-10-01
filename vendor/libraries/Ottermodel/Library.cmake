############################################################################
# Copyright 2013-2021 Norwegian University of Science and Technology (NTNU)#
# Department of Engineering Cybernetics (ITK)                              #
############################################################################
############################################################################
# Author: Nikolai Lauvås                                                   #
############################################################################

file(GLOB DUNE_OM_FILES
  user/vendor/libraries/Ottermodel/*.cpp)
set_source_files_properties(${DUNE_OM_FILES}
PROPERTIES COMPILE_FLAGS "${DUNE_CXX_FLAGS} ${DUNE_CXX_FLAGS_STRICT}")

list(APPEND DUNE_VENDOR_FILES ${DUNE_OM_FILES})

set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries)
