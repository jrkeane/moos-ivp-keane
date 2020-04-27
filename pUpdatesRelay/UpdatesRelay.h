/************************************************************/
/*    NAME: Jim Keane                                              */
/*    ORGN: MIT                                             */
/*    FILE: UpdatesRelay.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef UpdatesRelay_HEADER
#define UpdatesRelay_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class UpdatesRelay : public CMOOSApp
{
 public:
   UpdatesRelay();
   ~UpdatesRelay();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
   bool handleIncoming();


   //utilities
   bool setVehicleName(std::string);

 private: // Configuration variables

   std::string m_apckf_update;
  
    // State variables

};

#endif
