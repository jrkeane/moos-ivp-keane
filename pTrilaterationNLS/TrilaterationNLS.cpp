/************************************************************/
/*    NAME: Nick Rypkema                                              */
/*    ORGN: MIT                                             */
/*    FILE: TrilaterationNLS.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "TrilaterationNLS.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/unsupported/Eigen/NonLinearOptimization"

using namespace std;

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

  int inputs() const { return m_inputs; }
  int values() const { return m_values; }

};

struct my_functor : Functor<double>
{
  my_functor(double _x1, double _y1, double _x2, double _y2, double _x3, double _y3, double _r1, double _r2, double _r3): Functor<double>(2,3) {
    x1 = _x1;
    y1 = _y1;
    x2 = _x2;
    y2 = _y2;
    x3 = _x3;
    y3 = _y3;
    r1 = _r1;
    r2 = _r2;
    r3 = _r3;
  }

  int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
  {
    fvec(0) = pow(x1-x(0), 2) + pow(y1-x(1), 2) - pow(r1, 2);
    fvec(1) = pow(x2-x(0), 2) + pow(y2-x(1), 2) - pow(r2, 2);
    fvec(2) = pow(x3-x(0), 2) + pow(y3-x(1), 2) - pow(r3, 2);

    return 0;
  }
  int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
  {
    fjac(0,0) = 2*x(0) - x1;
    fjac(0,1) = 2*x(1) - y1;
    fjac(1,0) = 2*x(0) - x2;
    fjac(1,1) = 2*x(1) - y2;
    fjac(2,0) = 2*x(0) - x3;
    fjac(2,1) = 2*x(1) - y3;

    return 0;
  }

  private:
    double x1, y1, x2, y2, x3, y3, r1, r2, r3;
};

//---------------------------------------------------------
// Constructor

TrilaterationNLS::TrilaterationNLS()
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

TrilaterationNLS::~TrilaterationNLS()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool TrilaterationNLS::OnNewMail(MOOSMSG_LIST &NewMail)
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

bool TrilaterationNLS::OnConnectToServer()
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

bool TrilaterationNLS::Iterate()
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

    Eigen::VectorXd x(2);
    x(0) = m_x;
    x(1) = m_y;

    my_functor functor(pinger1_x, pinger1_y, pinger2_x, pinger2_y, pinger3_x, pinger3_y, m_range1, m_range2, m_range3);
    Eigen::LevenbergMarquardt<my_functor> lm(functor);
    lm.minimize(x);

    m_x = x(0);
    m_y = x(1);

    m_new_range1 = false;
    m_new_range2 = false;
    m_new_range3 = false;
  }

  stringstream ss;
  ss << "NAME=zeta_TRILATNLS,TYPE=KAYAK,X=" << m_x << ",Y=" << m_y << ",HDG=" << m_nav_heading << ",SPD=" << m_nav_speed;
  ss << fixed << ",TIME=" << MOOSTime();
  cout << ss.str() << endl;

  Notify("NODE_REPORT", ss.str());

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool TrilaterationNLS::OnStartUp()
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

void TrilaterationNLS::RegisterVariables()
{
  // Register("FOOBAR", 0);
  Register("NAV_HEADING", 0.0);
  Register("NAV_SPEED", 0.0);
  Register("BRS_RANGE_REPORT", 0.0);
}
