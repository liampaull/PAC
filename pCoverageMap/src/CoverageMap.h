/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: CoverageMap.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef CoverageMap_HEADER
#define CoverageMap_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <probabilistic_coverage/XYCoverageMap.h>
#include <probabilistic_coverage/CoverageSensor.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/isam_coop.hpp>
#include <Eigen/LU>
#include <string>
#include "XYSegList.h"

using namespace std;

class CoverageMap : public AppCastingMOOSApp
{
 public:
   CoverageMap();
   ~CoverageMap();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void ReadMissionFile();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void update(bool post_update=true);
   void resetMap();
   void postFullMap();
   void updateMap(vector<int>);

 private: // lcm setup
   void lcm_subscribe();
   static void * lcm_handle(void* data);
   string m_provider;
   lcm::LCM* m_lcm;
   
 private: // lcm callbacks
   void on_pose(const lcm::ReceiveBuffer* rbuf,
		const string& channel,
		const coop::pose3_t* msg);

   void on_trajectory(const lcm::ReceiveBuffer* rbuf,
		      const string& channel,
		      const coop::trajectory_t* msg);

 private:
   int64_t sliding_window_test(const coop::trajectory_t*);
   double  entropy(Eigen::MatrixXd);

 private: // Configuration variables

   // vars used to turn sensor on or off
   double m_max_change_yaw; // in rads
   double m_max_move;
   int    m_id;
   bool   m_block_sensor_on_surface;
   double m_min_depth;
   XYPolygon         m_workspace_poly;
   double            m_sensor_resolution;
   double            m_sensor_range;
   vector<double>    m_sensor_func;
   double            m_mission_completion_value;
   string            m_mission_completion_criterion; //"mean"
   double            m_min_update;
   double            m_min_entropy_reduction;


 private: // State variables

   XYCoverageMap       m_coverage_map;
   CoverageSensor      m_coverage_sensor;
   coop::pose3_t       m_current_pose;
   coop::pose3_t       m_previous_pose;
   coop::trajectory_t  m_previous_trajectory;
   XYCoverageMap       m_previous_coverage_map;
   XYSegList           m_trajectory_seg;
   int64_t             m_prior_time;
   int                 m_points_updated;
   bool                m_current_pose_initialized;
   pthread_mutex_t     m_map_update_mutex;
   pthread_mutex_t     m_pose_update_mutex;

};
#endif 
