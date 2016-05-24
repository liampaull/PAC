/************************************************************/
/*    NAME: Liam Paull                                               */
/*    ORGN: MIT                                             */
/*    FILE: AdaptiveSurveyPlanner.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef AdaptiveSurveyPlanner_HEADER
#define AdaptiveSurveyPlanner_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYConvexGrid.h"
#include <string>

class AdaptiveSurveyPlanner : public AppCastingMOOSApp
{
 public:
   AdaptiveSurveyPlanner();
   ~AdaptiveSurveyPlanner() {};

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private:
   void initialize_new_leg();
   void shift_leg(double);
   void post_update(int currix=0);

 private: // Configuration variables
   double    m_survey_direction; // direction of the survey in degrees
   XYPolygon m_workspace_poly;
   double    m_mission_completion_value;
   double    m_leg_length;
   double    m_kp;
   int       m_id;
   double    m_look_ahead;

 private: // State variables
   double        m_effective_sensor_range;
   XYConvexGrid  m_coverage_map;
   double        m_nav_x;
   double        m_nav_y;
   double        m_current_leg_x1;
   double        m_current_leg_y1;
   double        m_current_leg_x2;
   double        m_current_leg_y2;
   std::string   m_leg; // "up" or "down"
   bool          m_first_leg;
   int           m_wpt_index;


 private: // for app casting
   double m_desired;
   double m_actual;
   double m_error; 
   bool   m_in_workspace;
   double m_sensor_range;
};

#endif 
