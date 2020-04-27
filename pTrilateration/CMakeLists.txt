#--------------------------------------------------------
# The CMakeLists.txt for:                       pTrilateration
# Author(s):                              Nick Rypkema
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  Trilateration.cpp
  Trilateration_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pTrilateration ${SRC})

TARGET_LINK_LIBRARIES(pTrilateration
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

