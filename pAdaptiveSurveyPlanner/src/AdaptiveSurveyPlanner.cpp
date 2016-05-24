/************************************************************/
/*    NAME: Liam Paull                                               */
/*    ORGN: MIT                                             */
/*    FILE: AdaptiveSurveyPlanner.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "AdaptiveSurveyPlanner.h"
#include "XYFormatUtilsConvexGrid.h"
#include "XYFormatUtilsPoly.h"
#include "XYPoint.h"
#include "GeomUtils.h"
#include "math.h" //trig functions

using namespace std;

//---------------------------------------------------------
// Constructor

AdaptiveSurveyPlanner::AdaptiveSurveyPlanner()
{
  m_first_leg = true;
  m_look_ahead = 10.0;
  m_wpt_index = 0;

}

//---------------------------------------------------------
// Procedure: OnNewMail

bool AdaptiveSurveyPlanner::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

     if(key == "EFFECTIVE_SENSOR_RANGE") 
       m_effective_sensor_range = dval;
     else if(key == "VIEW_GRID"){
       m_coverage_map = string2ConvexGrid(sval);
       m_coverage_map.set_label("Coverage_Map_" + intToString(m_id));
     }
     else if(key == "CONVEX_GRID_DELTA")
       m_coverage_map.processDelta(sval);
     else if(key == "NAV_X")
       m_nav_x = dval;
     else if(key == "NAV_Y")
       m_nav_y = dval;
     else if(key == "LEG"){ // comes from endflag of either up_leg_survey or down_leg_survey. I.e. we have just finished a leg and now we need to setup the next track line (leg)
       m_leg = sval;
       if (!m_first_leg)
	 initialize_new_leg();
       else
	 m_first_leg = false;
     }
     else if(key == "WPT_INDEX")
       m_wpt_index = int(dval);
     else if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool AdaptiveSurveyPlanner::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool AdaptiveSurveyPlanner::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (m_wpt_index == 0){ // wpt_index = 0 means we are moving between legs. don't do any updating of the trackline in this case.
    AppCastingMOOSApp::PostReport();
    return (true); // don't do the place we are looking is not in the workspace 
  }

  m_desired = m_effective_sensor_range; // this is published by pCoverageMap and takes into account platform uncertainty
  m_actual = 0.0;
  double x;
  double y;
  // slight hack here. I'm going to propagate the nav_x nav_y values forward (in time) so that in the case when the veheicle is not exactly parallel to the desired track (specifically when the track is moving "out" (further away), this search isn't "fooled" into thinking that we are closer to the covered patch than we actually are. A picture describes it better...


  double look_x,look_y;
  if (m_leg == "up"){
    look_x = m_nav_x + m_look_ahead*sin(m_survey_direction - PI/2);
    look_y = m_nav_y + m_look_ahead*cos(m_survey_direction - PI/2);
  }
  else{ // (m_leg == "down"){
    look_x = m_nav_x + m_look_ahead*sin(m_survey_direction + PI/2);
    look_y = m_nav_y + m_look_ahead*cos(m_survey_direction + PI/2);
  }
  

  double look2_x, look2_y;
  look2_x = look_x + m_sensor_range*sin(m_survey_direction + PI);
  look2_y = look_y + m_sensor_range*cos(m_survey_direction + PI);

  XYPoint look1(look_x,look_y);
  look1.set_label("Look1_"+intToString(m_id));
  look1.set_vertex_size(3);
  string look1_str = look1.get_spec();
  //Notify("VIEW_POINT",look1_str);

  XYPoint look2(look2_x,look2_y);
  look1.set_label("Look2_"+intToString(m_id));
  look1.set_vertex_size(3);
  string look2_str = look2.get_spec();
  //Notify("VIEW_POINT",look2_str);


  if (m_workspace_poly.contains(look_x,look_y) || m_workspace_poly.contains(look2_x,look2_y))
    m_in_workspace = true;
  else{
    m_in_workspace = false;
    AppCastingMOOSApp::PostReport();
    return (true); // don't do the place we are looking is not in the workspace 
  }


  perpLineIntPt(m_current_leg_x1,m_current_leg_y1,
		m_current_leg_x2,m_current_leg_y2,
	        look_x,look_y,
		x,y); // project nav_x, nav_y onto the current track line
  int index = m_coverage_map.find_index_from_point(y,x);
  double coverage_val = m_coverage_map.getVal(index);
  double cell_size = m_coverage_map.getCellSize();
  bool in_workspace = false;

  while(coverage_val < m_mission_completion_value){
    if (in_workspace && !m_workspace_poly.contains(x,y))
      break; // we have moved out of the workspace
    in_workspace = m_workspace_poly.contains(x,y);

    x += cell_size*sin(m_survey_direction + PI);
    y += cell_size*cos(m_survey_direction + PI);

    index = m_coverage_map.find_index_from_point(y,x);
    coverage_val = m_coverage_map.getVal(index);
    m_actual += cell_size;
  }

  if (m_actual == 0.0){ // never went into the while loop. Means the track line is sitting somewhere that is actually covered.
    cout << "never went into while loop at all" << endl;
    m_actual = -m_effective_sensor_range;
  }

  m_error = m_desired - m_actual;

  shift_leg(m_kp*m_error);

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool AdaptiveSurveyPlanner::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = toupper(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "WORKSPACE_COORDS") {
      m_workspace_poly = string2Poly(value);
      handled = true;
    }
    else if(param == "SURVEY_DIRECTION") {
      m_survey_direction = atof(value.c_str())*PI/180;
      handled = true;
    }
    else if(param == "MISSION_COMPLETION_VALUE") {
      m_mission_completion_value = atof(value.c_str());
      handled = true;
    }
    if(param == "SENSOR_RANGE"){
      m_sensor_range = atof(value.c_str());
      handled = true;
    }
    else if(param == "START_LEG"){
      m_leg = tolower(value);
      handled = true;
    }
    else if(param == "LEG_LENGTH"){
      m_leg_length = atof(value.c_str());
      handled = true;
    }
    else if(param == "START_X1"){
      m_current_leg_x1 = atof(value.c_str());
      handled = true;
    }
    else if(param == "START_Y1"){
      m_current_leg_y1 = atof(value.c_str());
      handled = true;
    }
    else if(param == "START_X2"){
      m_current_leg_x2 = atof(value.c_str());
      handled = true;
    }
    else if(param == "START_Y2"){
      m_current_leg_y2 = atof(value.c_str());
      handled = true;
    }
    else if(param == "K_P"){
      m_kp = atof(value.c_str());
      handled = true;
    }
    else if(param == "VEHICLE_ID"){
      m_id = atoi(value.c_str());
      handled = true;
    }
    else if(param == "LOOK_AHEAD"){
      m_look_ahead = atof(value.c_str());
      handled=true;
    }
    if(!handled)
      reportUnhandledConfigWarning(orig);

    

  }

  post_update();
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void AdaptiveSurveyPlanner::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("EFFECTIVE_SENSOR_RANGE","*", 0);
  Register("VIEW_GRID","*", 0);
  Register("CONVEX_GRID_DELTA","*", 0);
  Register("NAV_X","*", 0);
  Register("NAV_Y","*", 0);
  Register("LEG","*", 0);
  Register("WPT_INDEX","*",0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool AdaptiveSurveyPlanner::buildReport() 
{
  m_msgs << "============================================ \n";
  if(!m_in_workspace)
    m_msgs << "No leg adjustments since outside of workspace \n";
  else{
    m_msgs << "Effective sensor range (\"desired\" distance) " + doubleToString(m_desired) + "\n";
    m_msgs << "Distance to nearest covered cell (or edge of workspace) (\"actual\" distance) " + doubleToString(m_actual) + "\n";
    m_msgs << "Adjust leg by error = " + doubleToString(m_kp*m_error) + "\n";
  }
  return(true);
}

void AdaptiveSurveyPlanner::initialize_new_leg()
{

  if(m_leg != "up" && m_leg != "down"){
    reportRunWarning("Invalid leg type: " + m_leg + "\n");
    return;
  }

  m_current_leg_x1 = m_nav_x + 2*m_effective_sensor_range*sin(m_survey_direction);
  m_current_leg_y1 = m_nav_y + 2*m_effective_sensor_range*cos(m_survey_direction);

  if (m_leg == "up"){
    m_current_leg_x2 = m_current_leg_x1 + 
      m_leg_length*sin(m_survey_direction - PI/2);
    m_current_leg_y2 = m_current_leg_y1 + 
      m_leg_length*cos(m_survey_direction - PI/2);
  }
  else{
    m_current_leg_x2 = m_current_leg_x1 + 
      m_leg_length*sin(m_survey_direction + PI/2);
    m_current_leg_y2 = m_current_leg_y1 + 
      m_leg_length*cos(m_survey_direction + PI/2);
  }

  reportEvent("Starting new leg: " + m_leg + "\n"); 

  post_update();
}

void AdaptiveSurveyPlanner::shift_leg(double shift_val)
{
  m_current_leg_x1 = m_current_leg_x1 + shift_val*sin(m_survey_direction);
  m_current_leg_y1 = m_current_leg_y1 + shift_val*cos(m_survey_direction);
  m_current_leg_x2 = m_current_leg_x2 + shift_val*sin(m_survey_direction);
  m_current_leg_y2 = m_current_leg_y2 + shift_val*cos(m_survey_direction);

  post_update(1);
  //  if (m_leg == "up")
  //    Notify("UP_LEG_UPDATES","currix=1");
  //  else
  //    Notify("DOWN_LEG_UPDATES","currix=1");

}

void AdaptiveSurveyPlanner::post_update(int currix)
{
  //build the updates string:
  string update = "points=" + 
    doubleToString(m_current_leg_x1) + "," + 
    doubleToString(m_current_leg_y1) + ":" + 
    doubleToString(m_current_leg_x2) + "," + 
    doubleToString(m_current_leg_y2);
  if (currix)
    update += "# currix=" + intToString(currix);
  
  // post
  if (m_leg == "up")
    Notify("UP_LEG_UPDATES",update);
  else
    Notify("DOWN_LEG_UPDATES",update);
}
