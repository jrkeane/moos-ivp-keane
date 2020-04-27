/************************************************************/
/*    NAME: Jim Keane                                              */
/*    ORGN: AMC                                             */
/*    FILE: HomeToLBL.cpp                                        */
/*    DATE:                                                 */
/************************************************************/


#include <iterator>
#include "MBUtils.h"
#include "HomeToLBL.h"
#include <cmath>
#include <sstream>
#include <vector>
#include <fstream>


using namespace std;

//---------------------------------------------------------
// Constructor

HomeToLBL::HomeToLBL()
{
  m_iterations = 0;
  m_timewarp   = 1;
  m_rr         = 0; 
  broadcast    = 0;
  //m_beacon_depth = 1 ; //lbl beacon remains 1m below surface.
  m_ii = 1;
  m_backup = 1;
  m_recovery_x = 0;
  m_recovery_y = 200;
  m_homing_rr = 0;
  m_attempt = 0;
  m_ready_to_home = 0;
  m_range = 0;
  m_no_more = 0;
  
}

//---------------------------------------------------------
// Destructor

HomeToLBL::~HomeToLBL()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool HomeToLBL::OnNewMail(MOOSMSG_LIST &NewMail)
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

// handles incoming communications from the MOOSDB 

  if (MOOSStrCmp(msg.GetKey(), "NAV_X")) {
    m_nav_x = msg.GetDouble();
    m_nav_rec++;

  }else if (MOOSStrCmp(msg.GetKey(), "NAV_Y")) {
    m_nav_y = msg.GetDouble();

  }else if (MOOSStrCmp(msg.GetKey(), "NAV_DEPTH")) {
    m_nav_depth = msg.GetDouble();

  }else if (MOOSStrCmp(msg.GetKey(), "CLOSING")) { // behaviour status  
m_closing = msg.GetString(); 
m_attempt++;
/*if (m_closing=="true"){
m_attempt=1;
}else{
m_attempt=0;
}*/

 }else if (MOOSStrCmp(msg.GetKey(), vehString.str())){ //this is the range report
    m_brs_range_report = msg.GetString();
    string m_range_report = tokStringParse(m_brs_range_report, "range", ',', '=');

    istringstream om(m_range_report);
//this is where the localisation algorithms are employed

//first assign all the variables from the latest MOOSDB reports
    om >> m_range;
    m_range = m_range * m_rr_factor;	
    m_x_pos = m_nav_x;
    m_y_pos = m_nav_y;
    m_z_depth = m_nav_depth;

//apply pythogoras to reduce localisation to a 2d plane

    Pythag_Algorithm(); 

// build arrays for trilateration
    
    v_x.push_back (m_x_pos);
    v_y.push_back (m_y_pos);
    v_depth.push_back (m_z_depth);
    v_r.push_back (pythag_aa);
    m_rr++;

//apply trilateration

    Trilat_Algorithm(); 

   if (m_backup <5){ //this statement stops broadcasting during recovery. 
if (broadcast==1){ //so if the trilateration results are valid.
    Median();
    Broadcast(); 

    CloseRange(); 

} 
}
 } else if (MOOSStrCmp(msg.GetKey(), "HOMING")) { // behaviour status
    m_homing = msg.GetString();
    m_Comms.Notify("CLOSER", m_homing);
    m_ready_to_home=1;
if (m_homing=="true") { // if in the homing phase then run the backup behaviour to make sure AUV is converging
       //backup_bhv();
//if (m_range<100){	

    	
//	}else{
	//backup_bhv();
//return (true);

}
 } else if (MOOSStrCmp(msg.GetKey(), "DEPLOY")) { // deploy is true while auv is in action.
    m_deploy = msg.GetString();
  
  if ((m_deploy=="false")&&(m_rr>0)){ //if mission has ended. Needs to have received at least one range report.. 
  if ((m_range > 150)&&(m_backup<5)){ // homing failed
        //jk_recovery();
      }  else { //homing complete
   //jk_logging(); // log to text file
}
}
return(true);
}
}
}
//---------------------------------------------------------
// Procedure: OnConnectToServer          no idea what this one's about

bool HomeToLBL::OnConnectToServer()
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

bool HomeToLBL::Iterate()
{
  m_iterations++;

  std::stringstream publishString;
  
  publishString.str("");
  publishString.clear();
  
  string m_range_report = tokStringParse(m_brs_range_report, "range", ',', '=');
  //cout << "current range =" << m_range_report << endl; 
  publishString << "m_rr="<< m_rr << ",current range=" << m_range_report ;
  m_Comms.Notify("M_RANGE", publishString.str());
  
  publishString.str("");
  publishString.clear();
  
  //publishString << "AUV stats, mrr=" << m_rr << ", x2=" << x2 << "y2=" << y2 << "r2=" << r2 ;
  //m_Comms.Notify("TRILAT", publishString.str()); // broadcast latest AUV position and range report 

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool HomeToLBL::OnStartUp()
{
CMOOSApp::OnStartUp();

  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p); 

      if(param == "VEHICLE_NAME") {
        m_veh_name = value ; //handled
	vehString << "BRS_RANGE_REPORT_" << m_veh_name ;
  	m_Comms.Notify("VIEW_POINT", vehString.str());  // broadcast to make sure mission name handle is correct. 
      } else if(param == "BEACON_DEPTH") {
        m_beacon = value ;  //handled
        istringstream omm(m_beacon);
	omm >> m_beacon_depth;
	//m_beacon_depth = std::stod (m_beacon, 2) ;
      } else if(param == "RR_CORRECTION_FACTOR") {
        m_rr_correction = value ;  //handled
        istringstream ommm(m_rr_correction);
    	ommm >> m_rr_factor;
	//m_beacon_depth = std::stod (m_beacon, 2) ;
      } else if(param == "CLOSING_RANGE_DEMANDED") {
        m_closing_range_demand = value ;  //handled
        istringstream ommmm(m_closing_range_demand);
    	ommmm >> m_closing_demand;
	//m_beacon_depth = std::stod (m_beacon, 2) ;
      }
    }
  }
/*std:stringstream trilated;
trilated << "points=-200,100";
m_Comms.Notify("HOMING_UPDATES", trilated.str());  
*/
  m_timewarp = GetMOOSTimeWarp();

  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void HomeToLBL::RegisterVariables()
{
  // m_Comms.Register("FOOBAR", 0);

  //m_Comms.Register("VEHICLE_NAME", 0);
  m_Comms.Register("NAV_X", 0);
  m_Comms.Register("NAV_Y", 0);
  m_Comms.Register(vehString.str(), 0);
  m_Comms.Register("NAV_DEPTH", 0);
  m_Comms.Register("HOMING", 0);
  m_Comms.Register("DEPLOY", 0);
  m_Comms.Register("CLOSING", 0);
}

//-----------------------------------------------------------
// Procedure: Pythagoras. Apply to LBL range updates to reduce to a 2d search. 
bool HomeToLBL::Pythag_Algorithm()
{
    pythag_bb = (m_beacon_depth - m_z_depth) ;
    pythag_cc = m_range ;
    pythag_aa = sqrt((pythag_cc*pythag_cc)-(pythag_bb*pythag_bb)) ; // apply pythagoras to reduce to a 2d scenario. 

}
//-----------------------------------------------------------
// Procedure: trilateration. Estimate beacon location based on LBL range updates
bool HomeToLBL::Trilat_Algorithm()
{

if (m_rr>3)
{

//while (nnn<(m_rr-1)) {
//nnn=1;
n=1;
while (n<(m_rr-3)) {

//create variables from MOOSDB comms. 
	EPSILON = 1;

	x0=v_x[(m_rr-2-n)];
	y0=v_y[(m_rr-2-n)];
	r0=v_r[(m_rr-2-n)];
	x1=v_x[(m_rr-2)];
	y1=v_y[(m_rr-2)];
	r1=v_r[(m_rr-2)];
	x2=v_x[m_rr-1];
	y2=v_y[m_rr-1];
	r2=v_r[m_rr-1];

// Apply trilateration Algorithm

    /* dx and dy are the vertical and horizontal distances between
    * the circle centers.
    */
    dx = x1 - x0;
    dy = y1 - y0;

    /* Determine the straight-line distance between the centers. */
    d = sqrt((dy*dy) + (dx*dx));

    /* Check for solvability. */
    if (d > (r0 + r1))
    {
        /* no solution. circles do not intersect. */
        return false;
    }
    if (d < abs(r0 - r1))
    {
        /* no solution. one circle is contained in the other */
        return false;
    }

    /* 'point 2' is the point where the line through the circle
    * intersection points crosses the line between the circle
    * centers.
    */

    /* Determine the distance from point 0 to point 2. */
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

    /* Determine the coordinates of point 2. */
    point2_x = x0 + (dx * a/d);
    point2_y = y0 + (dy * a/d);

    /* Determine the distance from point 2 to either of the
    * intersection points.
    */
    h = sqrt((r0*r0) - (a*a));

    /*  determine the offsets of the intersection points from
    * point 2.
    */
    rx = -dy * (h/d);
    ry = dx * (h/d);

    /* Determine the absolute intersection points. */
    double intersectionPoint1_x = point2_x + rx;
    double intersectionPoint2_x = point2_x - rx;
    double intersectionPoint1_y = point2_y + ry;
    double intersectionPoint2_y = point2_y - ry;

    cout << "INTERSECTION Circle1 AND Circle2:(" << intersectionPoint1_x << "," << intersectionPoint1_y << ")" << " AND (" << intersectionPoint2_x << "," << intersectionPoint2_y << ")" << endl;

/* determine if circle 3 intersects at either of the above intersection points. */
    dx = intersectionPoint1_x - x2;
    dy = intersectionPoint1_y - y2;
    double d1 = sqrt((dy*dy) + (dx*dx));

    dx = intersectionPoint2_x - x2;
    dy = intersectionPoint2_y - y2;
    double d2 = sqrt((dy*dy) + (dx*dx));


/*determine validity of trilateration. 
if valid then broadcast =1 & the results will be broadcast 
back into MOOSDB. */

    if(abs(d1 - r2) < EPSILON) { //interesect occurs at point 1
  intersection_point_x = intersectionPoint1_x;
  intersection_point_y = intersectionPoint1_y;
  broadcast = 1; 
    }

    else if(abs(d2 - r2) < EPSILON) { //intersect occurs at point 2
  intersection_point_x = intersectionPoint2_x;
  intersection_point_y = intersectionPoint2_y;
  broadcast = 1;
    }

    else { //no intersect occurs
  cout << "INTERSECTION Circle1 AND Circle2 AND Circle3:NONE" << endl;
  broadcast = 0;
    }

return true ;
++n;
}
}
}

//----------------------------------------------------------------------
// Procedure: Apply median filter 

bool HomeToLBL::Median()
{
 
/* broadcast trilateration result to uXMS for user notification. */
  std:stringstream trilated;
  trilated << "x=" << intersection_point_x << ",y=" << intersection_point_y << "," << "active=true,label=" << intersection_point_x << " " << intersection_point_y << ",label_color=yellow,vertex_color=yellow,vertex_size=6" ;
  m_Comms.Notify("VIEW_POINT", trilated.str());
 

  std::stringstream publishTrilats;
  publishTrilats << "AUV stats, mrr=" << m_rr << ",trilat=" << intersection_point_x << "," << intersection_point_y;
  m_Comms.Notify("TRILAT", publishTrilats.str()); // broadcast latest AUV position and range report 
/* apply median filters */

Beacon_x.push_back (intersection_point_x); // create vectors of all beacon position estimates
Beacon_y.push_back (intersection_point_y);

/* apply median filter to x vector */

  size_x = Beacon_x.size();

  sort(Beacon_x.begin(), Beacon_x.end());

  if (size_x  % 2 == 0)
  {
      median_x = (Beacon_x[size_x / 2 - 1] + Beacon_x[size_x / 2]) / 2;
  }
  else 
  {
      median_x = Beacon_x[size_x / 2];
  }

intersection_point_x = median_x;

/*apply median filter to y vector */

  double median_y;
  size_t size_y = Beacon_y.size();

  sort(Beacon_y.begin(), Beacon_y.end());

  if (size_y  % 2 == 0)
  {
      median_y = (Beacon_y[size_y / 2 - 1] + Beacon_y[size_y / 2]) / 2;
  }
  else 
  {
      median_y = Beacon_y[size_y / 2];
  }

intersection_point_y = median_y;

}

//----------------------------------------------------------------------
// Procedure: Broadcasting. broadcast results to Helm & pMarineViewer 

bool HomeToLBL::Broadcast()
{
/* now that median filter has been applied - broadcast results. */

//broadcast to pMarineviewer for user notification

std::stringstream filtered;
filtered << "x=" << intersection_point_x << ",y=" << intersection_point_y << "," << "active=true,label=" << intersection_point_x << " " << intersection_point_y << ",label_color=red,vertex_color=red,vertex_size=5" ;
  m_Comms.Notify("VIEW_POINT", filtered.str());

//broadcast to homing wpt. 

 std::stringstream turnUpdates;

  turnUpdates << "points=" << intersection_point_x << "," << intersection_point_y ;
  
  m_Comms.Notify("HOMING_UPDATES", turnUpdates.str());

broadcast = 0; // reset default to not broadcast a homing point unless a valid result . 

    return true;
}

//--------------------------------------------------------------------
// Procedure: Close range approach to beacon. 

bool HomeToLBL::CloseRange()
{

   // std::stringstream homingString;
   // homingString << "points=" << intersection_point_x << "," << intersection_point_y ;
   // m_Comms.Notify("RETURN_UPDATES", homingString.str());
    //m_Comms.Notify("RETURN", "true");
    //m_Comms.Notify("HOMING", "false");
    
int closest = distance(v_r.begin(), min_element(v_r.begin(), v_r.end()));
double closest_r = v_r[closest];
std::stringstream closest_range_reported;
closest_range_reported << closest_r;
std::stringstream closingStatus;
closingStatus << "closest rr=" << closest_r;
m_Comms.Notify("CLOSER", closingStatus.str());

std::stringstream closeUpdates;

if ((closest_r<m_closing_demand)&&(m_no_more<1)){

double closestx = v_x[closest];
double closesty = v_y[closest];
double conf_xdist = closestx-intersection_point_x;
double conf_ydist = closesty-intersection_point_y;
double confidence_distance = sqrt((conf_xdist*conf_xdist)+(conf_ydist*conf_ydist));
std::stringstream confidence;
confidence << confidence_distance;
m_Comms.Notify("CONFIDENCE", confidence.str());

if (confidence_distance<(m_closing_demand+10)){
closeUpdates << "points=" << intersection_point_x << "," << (intersection_point_y-50) << ":" << intersection_point_x << "," << (intersection_point_y-5);
}else{
  closeUpdates << "points=" << closestx << "," << (closesty-50) << ":" << closestx << "," << (closesty-5);
}

   m_Comms.Notify("RETURN_UPDATES", closeUpdates.str());

   m_Comms.Notify("CLOSING", "false");
   m_Comms.Notify("RETURN", "true");
   m_Comms.Notify("HOMING", "false");

   closest_range_reported <<" at" << closestx << "," << closesty;
   m_Comms.Notify("FINAL_RANGE", closest_range_reported.str());
 
   std::stringstream turnUpdates;
   turnUpdates << "points=" << intersection_point_x << "," << intersection_point_y ;
   m_Comms.Notify("FINAL_TRILAT", turnUpdates.str());

   m_no_more=2;
  std::stringstream readyToHome;
   readyToHome << m_no_more ;
   m_Comms.Notify("READY_TO_HOME", readyToHome.str());
}else{

double dance;
double ipx;
double ipy;

if (closest_r>40){
dance = closest_r/2;
ipx = intersection_point_x;
ipy = intersection_point_y;
}else{
dance = 20;
if (m_attempt<5){
ipx = intersection_point_x;
ipy = intersection_point_y;
}else{
ipx = v_x[closest];
ipy = v_y[closest];
}
}
double adx1 = ipx + dance ;
double adx2 = ipx - dance ;
double adx3 = ipx + dance/2;
double adx4 = ipx - dance/2;
double ady1 = ipy + dance ;
double ady2 = ipy - dance ;
double ady3 = ipy + dance/2;
double ady4 = ipy - dance/2;

if ((m_nav_y<=ipy)&&(m_nav_x>ipx)){ // bottom right quadrant
  closeUpdates << "points=" << ipx << "," << ipy << ":" << ipx << "," << ady3 << ":" <<adx2 << "," << ady1 << ":" << adx2 << "," << ady2 << ":" <<ipx << "," << ady4 << ":" << ipx << "," << ady3 << ":"<< adx1 << "," << ady1 << ":" << adx1 << "," << ady2  ;
}else if ((m_nav_y>ipy)&&(m_nav_x>ipx)) { //top right quadrant
  closeUpdates << "points=" << ipx << "," << ipy << ":" << ipx << "," << ady4 << ":" <<adx2 << "," << ady2 << ":" << adx2 << "," << ady1 << ":" <<ipx << "," << ady3 << ":" << ipx << "," << ady4 << ":"<< adx1 << "," << ady2 << ":" << adx1 << "," << ady1  ;
}else if ((m_nav_x<=ipx)&&(m_nav_y>ipy)){ // coming from top left 
  closeUpdates << "points=" << ipx << "," << ipy << ":" << ipx << "," << ady4 << ":" <<adx1 << "," << ady2 << ":" << adx1 << "," << ady1 << ":" <<ipx << "," << ady3 << ":" << ipx << "," << ady4 << ":"<< adx2 << "," << ady2 << ":" << adx2 << "," << ady1  ;
}else if ((m_nav_x<=ipx)&&(m_nav_y<=ipy)) {  // coming from bottom left 
  closeUpdates << "points=" << ipx << "," << ipy << ":" << ipx << "," << ady3 << ":" <<adx1 << "," << ady1 << ":" << adx1 << "," << ady2 << ":" <<ipx << "," << ady4 << ":" << ipx << "," << ady3 << ":"<< adx2 << "," << ady1 << ":" << adx2 << "," << ady2  ;
}


if (m_closing=="false"){
//closeUpdates << "points=" << ipx << "," << ipy << ":" << adx1 << "," << ady3 << ":" << adx3 << "," << ady1 << ":" << adx4 << "," << ady1 << ":" << adx2 << "," << ady3 << ":" << adx2 << "," << ady4 << ":" << adx4 << "," << ady2 << ":" << adx3 << "," << ady2 << ":" << adx1 << "," << ady4 << ":" << adx1 << "," << ady3;
  m_Comms.Notify("CLOSING_UPDATES", closeUpdates.str());
 
}else{

  m_Comms.Notify("HOMING_UPDATES", closeUpdates.str());

}
  m_Comms.Notify("CLOSER", closeUpdates.str());
  m_Comms.Notify("CLOSEST_RANGE", closest_range_reported.str());
   //m_Comms.Notify("RETURN", "false");
   //m_Comms.Notify("HOMING", "true");
}
}

//--------------------------------------------------------------------
// Procedure: logging: log final results to custom text file. 

bool HomeToLBL::jk_logging()
{
	//m_Comms.Notify("RETURN", "true");
	//m_Comms.Notify("HOMING", "false");
	//m_Comms.Notify("DEPLOY", "true");

	std::stringstream finalpoint;
if (m_backup>5){
finalpoint << intersection_point_x << "," << intersection_point_y<< "," << "deemed fail, recovered at " << m_recovery_x << "," << m_recovery_y;
}else{
  	finalpoint << intersection_point_x << "," << intersection_point_y << "," << m_range ;
}
	m_Comms.Notify("TRILAT", finalpoint.str()) ;

//save to custom text log file
ofstream results;
  results.open ("/home/jimi/Desktop/buckhunter.txt", std::ios_base::app);
  results << finalpoint.str() << endl;
  results.close();

m_ii=3; 

return true;
}
//--------------------------------------------------------------------
// Procedure: BBehaviour. backupbehaviour in case homing's unsuccessful first time.

bool HomeToLBL::backup_bhv()
{

int approach1 = v_r[m_rr];
int approach2 = v_r[m_rr-1];
int approach3 = v_r[m_rr-2];
if ((approach1>approach2)&&(approach2>approach3)){ //this means auv is moving away from beacon
 if (m_backup==1){
backup_x = intersection_point_x;
backup_y = Beacon_y[round(size_y*3/4)];

}else if (m_backup==2){
backup_x = intersection_point_x;
backup_y = Beacon_y[round(size_y/4)];

}else if (m_backup==3){
backup_x = Beacon_x[round(size_x/4)];
backup_y = intersection_point_y;

}else if (m_backup==4){
 backup_x = Beacon_x[round(size_x*3/4)];
 backup_y = intersection_point_y;

}else if (m_backup>4){ 

backup_x = m_recovery_x;
backup_y = m_recovery_y;

}

std::stringstream turnUpdates;
  turnUpdates << "points=" << backup_x << "," << backup_y ;
  m_Comms.Notify("RETURN_UPDATES", turnUpdates.str());

++m_backup;
}
return true;
}  

//--------------------------------------------------------------------
// Procedure: recovery command.

bool HomeToLBL::jk_recovery()
{
 
std::stringstream turnUpdates;
  turnUpdates << "points=" << m_recovery_x << "," << m_recovery_y ;

  m_Comms.Notify("RETURN_UPDATES", turnUpdates.str());
  m_Comms.Notify("DEPLOY", "true");
  m_Comms.Notify("HOMING", "true");
  m_backup=10;

return true;
}  
