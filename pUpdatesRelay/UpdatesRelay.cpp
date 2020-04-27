/************************************************************/
/*    NAME: Jim Keane                                              */
/*    ORGN: MIT                                             */
/*    FILE: UpdatesRelay.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "UpdatesRelay.h"
#include <sstream>


using namespace std;

//---------------------------------------------------------
// Constructor

UpdatesRelay::UpdatesRelay()
{
  

}

//---------------------------------------------------------
// Destructor

UpdatesRelay::~UpdatesRelay()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool UpdatesRelay::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;

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

  string key = msg.GetKey();
  double dval = msg.GetDouble();
  string sval = msg.GetString();

  if (MOOSStrCmp(msg.GetKey(), "APCKF_PING")){

    m_apckf_update = msg.GetString();
    m_Comms.Notify("APCKF_UPDATES", m_apckf_update);
	} 
}
 return(true);
}


//---------------------------------------------------------
// Procedure: OnConnectToServer

bool UpdatesRelay::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool UpdatesRelay::Iterate()
{

  // m_Comms.Notify("COUNTER", v_size);

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool UpdatesRelay::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string line  = *p;
      string param = tolower(biteStringX(line, '='));
      string value = line;

      cout << param << endl;
      if(param == "foo") {
        //handled
      } else if(param == "bar") {
        //handled
      } 
    }
  }

  RegisterVariables();

  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void UpdatesRelay::RegisterVariables()
{
  m_Comms.Register("APCKF_PING",0);
  // Register("FOOBAR", 0);
}

