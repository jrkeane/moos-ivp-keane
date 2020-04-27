/************************************************************/
/*    NAME: Nick Rypkema                                              */
/*    ORGN: MIT                                             */
/*    FILE: TrilaterationNLS.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef TrilaterationNLS_HEADER
#define TrilaterationNLS_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class TrilaterationNLS : public CMOOSApp
{
 public:
   TrilaterationNLS();
   ~TrilaterationNLS();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();

 private: // Configuration variables

 private: // State variables
   unsigned int m_iterations;
   double       m_timewarp;
   double       m_x;
   double       m_y;
   double       m_nav_heading;
   double       m_nav_speed;
   double       m_range1;
   double       m_range2;
   double       m_range3;
   bool         m_new_range1;
   bool         m_new_range2;
   bool         m_new_range3;
   double       m_prev_time;
   double       m_dt;
};

#endif
