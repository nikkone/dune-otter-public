############################################################################
# Copyright 2013-2021 Norwegian University of Science and Technology (NTNU)#
# Department of Engineering Cybernetics (ITK)                              #
############################################################################
# This file is part of DUNE: Unified Navigation Environment.               #
#                                                                          #
# Commercial Licence Usage                                                 #
# Licencees holding valid commercial DUNE licences may use this file in    #
# accordance with the commercial licence agreement provided with the       #
# Software or, alternatively, in accordance with the terms contained in a  #
# written agreement between you and Faculdade de Engenharia da             #
# Universidade do Porto. For licensing terms, conditions, and further      #
# information contact lsts@fe.up.pt.                                       #
#                                                                          #
# Modified European Union Public Licence - EUPL v.1.1 Usage                #
# Alternatively, this file may be used under the terms of the Modified     #
# EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md #
# included in the packaging of this file. You may not use this work        #
# except in compliance with the Licence. Unless required by applicable     #
# law or agreed to in writing, software distributed under the Licence is   #
# distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     #
# ANY KIND, either express or implied. See the Licence for the specific    #
# language governing permissions and limitations at                        #
# https://github.com/LSTS/dune/blob/master/LICENCE.md and                  #
# http://ec.europa.eu/idabc/eupl.html.                                     #
############################################################################
# Author: Nikolai Lauvås                                                   #
############################################################################

find_package(Eigen3)
#set_package_properties(Eigen3 PROPERTIES
#    URL "http://eigen.tuxfamily.org"
#    PURPOSE "A linear algebra library used throughout OMPL.")


if(NOT Eigen3_FOUND)
  set(DUNE_USING_EIGEN3 0)
  message("Eigen3 not found")
else()
include_directories("${EIGEN3_INCLUDE_DIR}")
  #dune_add_lib(${EIGEN_INCLUDE_DIRS})
  #message(${EIGEN3_LIBRARIES})
  #INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
  set(DUNE_USING_EIGEN3 1)
endif()
