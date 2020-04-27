#--------------------------------------------------------
# The CMakeLists.txt for:                       pTrilaterationNLS
# Author(s):                              Nick Rypkema
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  TrilaterationNLS.cpp
  TrilaterationNLS_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pTrilaterationNLS ${SRC})

TARGET_LINK_LIBRARIES(pTrilaterationNLS
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

