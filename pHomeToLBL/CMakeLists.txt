#--------------------------------------------------------
# The CMakeLists.txt for:                       pHomeToLBL
# Author(s):                              Jim Keane
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  HomeToLBL.cpp
  HomeToLBL_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pHomeToLBL ${SRC})

TARGET_LINK_LIBRARIES(pHomeToLBL
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

