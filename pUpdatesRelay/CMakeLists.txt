#--------------------------------------------------------
# The CMakeLists.txt for:                       pUpdatesRelay
# Author(s):                              Jim Keane
#--------------------------------------------------------

SET(SRC
  UpdatesRelay.cpp
  UpdatesRelay_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pUpdatesRelay ${SRC})

TARGET_LINK_LIBRARIES(pUpdatesRelay
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

