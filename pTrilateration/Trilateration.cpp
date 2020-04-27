/************************************************************/
/*    NAME: Nick Rypkema                                              */
/*    ORGN: MIT                                             */
/*    FILE: Trilateration.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "Trilateration.h"
#include <math.h>
#include <sstream>

using namespace std;

//---------------------------------------------------------
// Constructor

Trilateration::Trilateration()
{
  m_iterations = 0;
  m_timewarp   = 1;

  m_x = 0;
  m_y = -10;
  m_nav_heading = 0;
  m_nav_speed = 0;
  m_range1 = 0;
  m_range2 = 0;
  m_range3 = 0;
  m_new_range1 = false;
  m_new_range2 = false;
  m_new_range3 = false;
  m_prev_time = -1;
}

//---------------------------------------------------------
// Destructor

Trilateration::~Trilateration()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool Trilateration::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

    if(msg.GetKey() == "NAV_HEADING") {
      m_nav_heading = msg.GetDouble();
    } else if(msg.GetKey() == "NAV_SPEED") {
      m_nav_speed = msg.GetDouble();
    } else if(msg.GetKey() == "BRS_RANGE_REPORT") {
      string b_id = tokStringParse(msg.GetString(), "id", ',', '=');
      double id = atof(b_id.c_str());
      string b_r = tokStringParse(msg.GetString(), "range", ',', '=');
      double r = atof(b_r.c_str());
      if(id == 1) {
        m_range1 = r;
        m_new_range1 = true;
      } else if(id == 2) {
        m_range2 = r;
        m_new_range2 = true;
      } else if(id == 3) {
        m_range3 = r;
        m_new_range3 = true;
      }
    }

#if 0 // Keep these around just for template
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString();
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
   }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Trilateration::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
   // m_MissionReader.GetConfigurationParam("Name", <string>);
   // m_Comms.Register("VARNAME", 0);

   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Trilateration::Iterate()
{
  m_iterations++;

  if(m_prev_time == -1) m_prev_time = MOOSTime();
  double dt = MOOSTime() - m_prev_time;
  m_prev_time = MOOSTime();

  m_x = m_x + m_nav_speed*dt*sin(m_nav_heading*M_PI/180.0);
  m_y = m_y + m_nav_speed*dt*cos(m_nav_heading*M_PI/180.0);

  if(m_new_range1 && m_new_range2 && m_new_range3) {
    double pinger1_x = 300;
    double pinger1_y = 100;
    double pinger2_x = 100;
    double pinger2_y = -200;
    double pinger3_x = -300;
    double pinger3_y = -100;
    stringstream css;
    css << "x=" << pinger1_x << ",y=" << pinger1_y << ",radius=" << m_range1 << ",duration=30" << ",label=range_1,edge_color=white,fill_color=white,fill=0.1,edge_size=1";
    css << fixed << ",time=" << MOOSTime();
    Notify("VIEW_CIRCLE", css.str());
    css.str("");
    css << "x=" << pinger2_x << ",y=" << pinger2_y << ",radius=" << m_range2 << ",duration=30" << ",label=range_2,edge_color=white,fill_color=white,fill=0.1,edge_size=1";
    css << fixed << ",time=" << MOOSTime();
    Notify("VIEW_CIRCLE", css.str());
    css.str("");
    css << "x=" << pinger3_x << ",y=" << pinger3_y << ",radius=" << m_range3 << ",duration=30" << ",label=range_3,edge_color=white,fill_color=white,fill=0.1,edge_size=1";
    css << fixed << ",time=" << MOOSTime();
    Notify("VIEW_CIRCLE", css.str());

    double P2P1 = pow(pinger2_x - pinger1_x, 2) + pow(pinger2_y - pinger1_y, 2);

    double ex_x = (pinger2_x - pinger1_x)/sqrt(P2P1);
    double ex_y = (pinger2_y - pinger1_y)/sqrt(P2P1);

    double P3P1_x = pinger3_x - pinger1_x;
    double P3P1_y = pinger3_y - pinger1_y;

    double ivar = ex_x*P3P1_x + ex_y*P3P1_y;

    double P3P1i = pow(pinger3_x - pinger1_x - ex_x*ivar, 2) + pow(pinger3_y - pinger1_y - ex_y*ivar, 2);

    double ey_x = (pinger3_x - pinger1_x - ex_x*ivar)/sqrt(P3P1i);
    double ey_y = (pinger3_y - pinger1_y - ex_y*ivar)/sqrt(P3P1i);

    double d = sqrt(P2P1);

    double jvar = ey_x*P3P1_x + ey_y*P3P1_y;

    double x = (pow(m_range1, 2) - pow(m_range2, 2) + pow(d, 2))/(2*d);
    double y = ((pow(m_range1, 2) - pow(m_range3, 2) + pow(ivar, 2) + pow(jvar, 2))/(2*jvar)) - ((ivar/jvar)*x);

    m_x = pinger1_x + ex_x*x + ey_x*y;
    m_y = pinger1_y + ex_y*x + ey_y*y;

    m_new_range1 = false;
    m_new_range2 = false;
    m_new_range3 = false;
  }

  stringstream ss;
  ss << "NAME=zeta_TRILAT,TYPE=KAYAK,X=" << m_x << ",Y=" << m_y << ",HDG=" << m_nav_heading << ",SPD=" << m_nav_speed;
  ss << fixed << ",TIME=" << MOOSTime();
  cout << ss.str() << endl;

  Notify("NODE_REPORT", ss.str());

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Trilateration::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);

      if(param == "FOO") {
        //handled
      }
      else if(param == "BAR") {
        //handled
      }
    }
  }

  m_timewarp = GetMOOSTimeWarp();

  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void Trilateration::RegisterVariables()
{
  // Register("FOOBAR", 0);
  Register("NAV_HEADING", 0.0);
  Register("NAV_SPEED", 0.0);
  Register("BRS_RANGE_REPORT", 0.0);
}

