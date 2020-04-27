/************************************************************/
/*    NAME: Jim Keane                                              */
/*    ORGN: MIT                                             */
/*    FILE: HomeToLBL.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef HomeToLBL_HEADER
#define HomeToLBL_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <vector>

class HomeToLBL : public CMOOSApp
{
 public:
   HomeToLBL();
   ~HomeToLBL();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
   bool Pythag_Algorithm();
   bool Trilat_Algorithm();
   bool Median();
   bool Broadcast();
   bool jk_logging();
   bool backup_bhv();
   bool jk_recovery();
   bool CloseRange();

 //utilities
   bool setVehicleName(std::string);

 private: // Configuration variables
   
   unsigned int m_rr; //no of range reports rec'd
   unsigned int m_nav_rec; // no of navigation reports rec'd
   unsigned int m_ii;
   unsigned int m_ready_to_home;
   std::string m_veh_name;
   std::string m_brs_range_report;
   std::stringstream vehString;
   //std::string m_homingstatus;
   std::string m_deploy;
   std::string m_homing;
   std::string m_closing;
   double m_nav_x;
   double m_nav_y;
   double m_nav_depth;
   double m_x_pos;
   double m_y_pos;
   double m_z_depth;
   double m_heading;
   double m_speed;
   double pythag_aa ;
   double pythag_bb ;
   double pythag_cc ;

   std::string m_beacon ;
   double m_beacon_depth ; 
   std::string m_range_report;
   double m_range;
   std::vector<double> v_x ;
   std::vector<double> v_y ;
   std::vector<double> v_depth ;
   std::vector<double> v_r ;
   std::vector<double> Beacon_x;
   std::vector<double> Beacon_y;

   
       // Trilateration variables
    double n;
    double nnn;
    double EPSILON;
    double x0; 
    double y0; 
    double r0;
    double x1; 
    double y1; 
    double r1;
    double x2; 
    double y2; 
    double r2;
    
    double a; 
    double dx; 
    double dy;
    double d; 
    double h;
    double rx;
    double ry;
    double point2_x; 
    double point2_y;
    double intersection_point_x;
    double intersection_point_y;
    unsigned int broadcast;

       // median filter variables
    double median_x;
    double median_y;
    size_t size_x;
    size_t size_y;

 private: // State variables
   unsigned int m_iterations;
   double       m_timewarp;
   unsigned int m_backup;
   unsigned int m_homing_rr;
double backup_x;
double backup_y;
double m_recovery_x;
double m_recovery_y;
std::string m_rr_correction;
double m_rr_factor;
std::string m_closing_range_demand;
double m_closing_demand;
double m_no_more;
unsigned int m_attempt;
};

// advanced homing variables 
/*
double dance;
double ipx;
double ipy;

int closest ;
double closest_r;

double closestx ;
double closesty ;

double adx1 ;
double adx2 ;
double adx3 ;
double adx4 ;
double ady1 ;
double ady2 ;
double ady3 ;
double ady4 ;
*/
#endif 
