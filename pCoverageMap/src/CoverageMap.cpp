/************************************************************/
/*    NAME: Liam Paull                                              */
/*    ORGN: MIT                                             */
/*    FILE: CoverageMap.cpp                                        */
/*    DATE: Nov 6 2014
// Description: A total rewrite of previous pConfidenceMap application
// TODO: Gaps sometimes appear in coverage map upon the reception of the full trajectory                                                 */
/************************************************************/


#include "MBUtils.h"
#include "ACTable.h"
#include "XYFormatUtilsPoly.h"
#include "CoverageMap.h"
#include "isam_coop/conversions.h"
#include "MOOSApps/moos_utils.h"
#include "math.h"
#include <iterator>

#define e 2.718
// used for calculating entropy of Gaussians

//---------------------------------------------------------
// Constructor

CoverageMap::CoverageMap(){
  m_provider  = "udpm://239.255.76.56:7667"; //default can be overwritten
  m_block_sensor_on_surface = false;
  m_max_change_yaw = PI/6;
  m_max_move = 5;
  m_id = 1;
  m_ground_truth = false; // do we want to build a ground truth coverage map (i.e. from GPS data or simulation data)
  m_runtype = "simulation" ; // or "hover" -- affects what vars to subscribe to for GPS if in m_ground_truth
  m_points_updated = 0;
  m_mission_completion_value = 0.95;
  m_mission_completion_criterion = "mean";
  m_min_update = 0.1;
  m_min_entropy_reduction = 0.1;
  m_prior_time = 0;
  m_previous_trajectory.num_poses = 0;
  m_previous_trajectory.poses.resize(0);
  m_current_pose_initialized = false;
  pthread_mutex_init(&m_map_update_mutex, NULL);
  pthread_mutex_init(&m_pose_update_mutex, NULL);

}


CoverageMap::~CoverageMap()
{
  delete m_lcm;
}

//---------------------------------------------------------
// Procedure: OnNewMail
bool CoverageMap::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    if (m_runtype == "hover"){
      if(key == "RTK_X"){
	m_current_pose.mu[1] = dval;
	m_current_pose_initialized=true;
      }
      else if (key == "RTK_Y"){
	m_current_pose.mu[0] = dval;
	m_current_pose_initialized=true;
      }
      else if (key == "GPS_HEADING"){
	m_current_pose.mu[3] = dval*PI/180;
	m_current_pose_initialized=true;
      }
    }
    else if (m_runtype == "simulation"){
      if (key == "SIM_X"){
	m_current_pose.mu[1] = dval;
	m_current_pose_initialized=true;
      }
      else if (key == "SIM_Y"){
	m_current_pose.mu[0] = dval;
	m_current_pose_initialized=true;
      }
      else if (key == "SIM_HEADING"){
	m_current_pose.mu[3] = dval;
	m_current_pose_initialized=true;
      }
    }
    if (key == "SAVE_COVERAGE_MAP"){
      m_coverage_map.save(sval);
    }
  }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool CoverageMap::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//          happens AppTick times per second

bool CoverageMap::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (!m_current_pose_initialized){
    return(true);
  }
  if(pthread_mutex_trylock(&m_map_update_mutex)==0){
    update(); // update the coverage map
    if (m_ground_truth)
      post_trajectory_point();
    pthread_mutex_unlock(&m_map_update_mutex);
  }
  Notify("EFFECTIVE_SENSOR_RANGE",m_coverage_map.get_effective_range());
  
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool CoverageMap::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  ReadMissionFile();

  // initialize lcm stuff
  m_lcm = new lcm::LCM(m_provider.c_str());
  lcm_subscribe();
  pthread_t lcm_subscribe_thread;
  if(pthread_create(&lcm_subscribe_thread,NULL,CoverageMap::lcm_handle,this)){
    reportRunWarning("Error creating LCM subscribe thread");
  }

  // initialize coverage map stuff
  m_coverage_map.initialize(m_workspace_poly);
  m_coverage_map.set_label("Coverage_Map_" + intToString(m_id));
  m_coverage_map.set_criterion(m_mission_completion_criterion);
  m_coverage_map.set_completion_value(m_mission_completion_value);
  m_coverage_sensor.initialize(m_sensor_func, m_sensor_range, m_sensor_resolution);
  resetMap();
  m_previous_coverage_map = m_coverage_map;

  registerVariables();	
return(true);
}

void CoverageMap::ReadMissionFile()
{
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration("pCoverageMap", sParams))
    reportConfigWarning("No config block found for pCoverageMap");

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = toupper(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "LCM_PROVIDER") {
      m_provider = "udpm://239.255.76.56:" + value;
      handled = true;
    }
    if(param == "MAX_CHANGE_YAW") {
      m_max_change_yaw = atof(value.c_str());
      handled = true;
    }
    if(param == "MAX_MOVE"){
      m_max_move = atof(value.c_str());
      handled = true;
    }
    if(param == "BLOCK_SENSOR_ON_SURFACE"){
      m_block_sensor_on_surface = (tolower(value) == "true");
      handled = true;
    }
    if(param == "GROUND_TRUTH"){
      m_ground_truth = (tolower(value) == "true");
      handled = true;
    }
    if(param == "RUNTYPE"){
      m_runtype = tolower(value);
      handled = true;
    }
    if(param == "MISSION_COMPLETION_VALUE"){
      m_mission_completion_value = atof(value.c_str());
      handled = true;
    }
    if(param == "MISSION_COMPLETION_CRITERION"){
      m_mission_completion_criterion = value;
      handled = true;
    } // not doing anything with this currently. The the XYConvexGrid values should reflect the specified criterion
    if(param == "WORKSPACE_COORDS"){
      m_workspace_poly = string2Poly(value);
      Notify("VIEW_POLYGON", m_workspace_poly.get_spec());
      handled = true;
    }
    if(param == "SENSOR_RESOLUTION"){
      m_sensor_resolution = atof(value.c_str());
      handled = true;
    }
    if(param == "SENSOR_RANGE"){
      m_sensor_range = atof(value.c_str());
      handled = true;
    }
    if(param == "MIN_DEPTH"){
      m_min_depth = atof(value.c_str());
      handled = true;
    }
    if(param == "VEHICLE_ID"){
      m_id = atoi(value.c_str());
      handled = true;
    }
    if(param == "SENSOR_FUNCTION"){
      vector<string> str_vec = parseString(value,',');
      for(int i=0 ; i<str_vec.size(); i++){
	m_sensor_func.push_back(atof(str_vec[i].c_str()));
      }
      handled = true;
    }
    if(param == "MIN_UPDATE"){
      m_min_update = atof(value.c_str());
      handled = true;
    }
    if(param == "MIN_ENTROPY_REDUCTION"){
      m_min_entropy_reduction = atof(value.c_str());
      handled = true;
    }
  }
  m_trajectory_seg.set_label("trajectory_"+m_host_community);

  if (m_ground_truth){
    m_current_pose.mu[2] = 0.0;
    m_current_pose.mu[4] = 0.0;
    m_current_pose.mu[5] = 0.0;

    for (int i =0; i<6; i++)
      for (int j=0; j<6; j++)
	if (i==j){
	  m_previous_pose.sigma[i][j]=1;
	  m_current_pose.sigma[i][j]=1;
	}
	else{
	  m_previous_pose.sigma[i][j]=0.0;
	  m_current_pose.sigma[i][j]=0.0;
	}
  }

}  

//---------------------------------------------------------
// Procedure: registerVariables

 void CoverageMap::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  if (m_ground_truth){
    if(m_runtype =="simulation")
      Register("SIM_*","*", 0); //
    else if (m_runtype == "hover"){
      Register("RTK_*","*",0);
      Register("COMPASS_*","*",0);
      Register("GPS_*","*",0);
    }
  }
  Register("SAVE_COVERAGE_MAP","*",0);
}
 
 
//------------------------------------------------------------
// Procedure: buildReport()

bool CoverageMap::buildReport() 
{
  m_msgs << "============================================ \n";
  m_msgs << "Configs read:                                        \n";
  m_msgs << "Max move: " + doubleToString(m_max_move) + "\n";
  m_msgs << "Max change yaw: " + doubleToString(m_max_change_yaw) + "\n";
  m_msgs << "Block sensor on surface " + (m_block_sensor_on_surface ? string("true"):string("false")) + "\n";
  m_msgs << "Min depth: " + doubleToString(m_min_depth) + "\n";
  m_msgs << "Workspace poly" + m_workspace_poly.get_spec() + "\n";
  m_msgs << "Sensor resolution" + doubleToString(m_sensor_resolution) + "\n";
  m_msgs << "Sensor range" + doubleToString(m_sensor_range) + "\n";
  m_msgs << "============================================ \n\n";

  if(!m_current_pose_initialized)
    m_msgs << "awaiting pose initialization" << endl;
  else{

    m_msgs << "Current pose (x,y,z,yaw,pitch,roll) = ( " 
      + doubleToString(m_current_pose.mu[0]) + ","
      + doubleToString(m_current_pose.mu[1]) + ","
      + doubleToString(m_current_pose.mu[2]) + ","
      + doubleToString(m_current_pose.mu[3]) + ","
      + doubleToString(m_current_pose.mu[4]) + ","
      + doubleToString(m_current_pose.mu[5]) + ") \n\n";

    m_msgs << "Previous pose (x,y,z,yaw,pitch,roll) = ( " 
      + doubleToString(m_previous_pose.mu[0]) + ","
      + doubleToString(m_previous_pose.mu[1]) + ","
      + doubleToString(m_previous_pose.mu[2]) + ","
      + doubleToString(m_previous_pose.mu[3]) + ","
      + doubleToString(m_previous_pose.mu[4]) + ","
      + doubleToString(m_previous_pose.mu[5]) + ") \n\n";
  }
  m_msgs << "Points updated = " + intToString(m_points_updated) + "\n";

  return(true);
}
 
// start the infinite loop for lcm subscription 
 void* CoverageMap::lcm_handle(void* data)
{
  while(0==((CoverageMap *) data)->m_lcm->handle());
}
 
void CoverageMap::lcm_subscribe()
{
  m_lcm->subscribe(string("CURRENT_POSE"),&CoverageMap::on_pose,this);
  m_lcm->subscribe(string("TRAJECTORY"),&CoverageMap::on_trajectory,this);
}

void CoverageMap::post_trajectory_point(){
    XYPoint p(m_current_pose.mu[1], m_current_pose.mu[0]);
    m_trajectory_seg.add_vertex(p);
    string s = m_trajectory_seg.get_spec();
    Notify("VIEW_SEGLIST",s);
}  

void CoverageMap::on_pose(const lcm::ReceiveBuffer* rbuf,
			   const string& channel,
			   const coop::pose3_t* msg)
{
  if(m_ground_truth)
    return; // in ground_truth mode coverage map comes from MOOS vars

  // just update the current pose so that we can run the coverage map updating at Apptick frequency
  if(pthread_mutex_trylock(&m_pose_update_mutex)==0){
    m_current_pose  = *msg;
    post_trajectory_point();
    pthread_mutex_unlock(&m_pose_update_mutex);
  }
  if (!m_current_pose_initialized){
    m_previous_pose = m_current_pose;
    m_current_pose_initialized = true;
  }
}

void CoverageMap::on_trajectory(const lcm::ReceiveBuffer* rbuf,
				  const string& channel,
				  const coop::trajectory_t* msg)
{
  if (m_ground_truth)
    return; // no need to process trajectory in ground truth (we had the perfect data anyways)
  // lock both mutexs?
  pthread_mutex_lock(&m_map_update_mutex);
  pthread_mutex_lock(&m_pose_update_mutex);
  m_trajectory_seg.clear();
  m_trajectory_seg.set_label("trajectory_"+m_host_community);
  if (m_prior_time == 0) // first time
    m_prior_time = (msg->poses[0]).t;
  int64_t start_time = time_in_ms();
  // process the trajectory
  // step 1: reset the coverage: 
  m_coverage_map = m_previous_coverage_map;
  int64_t new_prior_time = sliding_window_test(msg);
  int64_t sliding_window_time = time_in_ms();
  reportEvent("old prior time = " + intToString(m_prior_time) + "new prior time = " + intToString(new_prior_time));
  m_previous_trajectory.num_poses = msg->num_poses;
  m_previous_trajectory.poses.resize(msg->num_poses);
  m_previous_trajectory.poses[0] = msg->poses[0];
  // step 2: cycle through the tractory and update 
  m_previous_pose = msg->poses[0];
  for (int i = 1; i<msg->num_poses ; i++){
    m_current_pose = msg->poses[i];
    if ((msg->poses[i]).t < m_prior_time)
      continue; // this is the key time saver here: skip update if before prior
    update(false);
    if ((msg->poses[i]).t == new_prior_time){
      reportEvent("Caching coverage map at time" + intToString(new_prior_time));
      m_previous_coverage_map = m_coverage_map;
    }
    m_previous_trajectory.poses[i] = msg->poses[i];
    XYPoint p((msg->poses[i]).mu[1], (msg->poses[i]).mu[0]);
    m_trajectory_seg.add_vertex(p);
  }
  m_prior_time = new_prior_time;
  int64_t int_time = time_in_ms();
  postFullMap();
  int64_t final_time = time_in_ms();
  reportEvent("time for sliding window test: " + intToString(sliding_window_time - start_time) +
	      "time to process traj: " + intToString(int_time-start_time) +
	      "time to post full map: " + intToString(final_time-int_time));

  pthread_mutex_unlock(&m_map_update_mutex);
  pthread_mutex_unlock(&m_pose_update_mutex);
}

int64_t CoverageMap::sliding_window_test(const coop::trajectory_t* new_traj){
  for(int i =0; i<m_previous_trajectory.num_poses;i++){
    if((m_previous_trajectory.poses[i]).t < m_prior_time)
      continue;
    Eigen::MatrixXd old_mu, new_mu, old_omega, new_omega;
    conversions::lcm2eigen(m_previous_trajectory.poses[i],old_mu,old_omega);
    conversions::lcm2eigen(new_traj->poses[i],new_mu,new_omega);
    Eigen::Vector2d old_pose_vector(old_mu(0), old_mu(1));
    Eigen::Vector2d new_pose_vector(new_mu(0), new_mu(1));
    double update_norm = (old_pose_vector - new_pose_vector).norm();
    double entropy_reduction = entropy(old_omega) - entropy(new_omega);
    //    reportEvent("update_norm = " + doubleToString(update_norm) + " entropy_reduction = " + doubleToString(entropy_reduction));
    if(update_norm > m_min_update || entropy_reduction > m_min_entropy_reduction)
      return (m_previous_trajectory.poses[i]).t;
  }
  return m_prior_time; // when in doubt don't update.
}

double CoverageMap::entropy(Eigen::MatrixXd omega){
  if(omega.rows() != omega.cols()){
    cout << "Cannot calculate entropy of non-square matrix" << endl;
    return 0.0;
  }
  return -0.5*log(pow(2*PI*e,omega.rows())*omega.determinant());
}

void CoverageMap::resetMap()
{
  m_coverage_map.reset(0.5);
  string spec = m_coverage_map.get_spec();
  Notify("VIEW_GRID",spec);
}


// TODO merge postFullMap and updateMap
void CoverageMap::postFullMap()
{
  // we are going to post it in big chunks to CONVEX_GRID_DELTA since
  // posting the entire thing to VIEW_GRID seems not to be working well
  int chunk_size = 100;
  int i = 0;
  int total_points = m_coverage_map.size();
  
  while (i*chunk_size < total_points) {
    string update_str = "Coverage_Map_" + intToString(m_id) + "@";
    for (int index = i*chunk_size; index < (i+1)*chunk_size; index ++){
      if (index > total_points) break;
      update_str += intToString(index) + "," + doubleToString(m_coverage_map.getVal(index)) + ":";
    }
    Notify("CONVEX_GRID_DELTA",update_str);
    i++;
  }
}

void CoverageMap::updateMap(vector<int> points)
{
  if(points.empty())
    return;
  int chunk_size = 100;
  int i = 0;
  while (i*chunk_size < points.size()){
    string update_str = "Coverage_Map_" + intToString(m_id) + "@";
    for (int index = i*chunk_size; index < (i+1)*chunk_size ; index++){
      if (index > points.size()) break;
      update_str += intToString(points[index]) + "," + doubleToString(m_coverage_map.getVal(points[index])) + ":";
    }
    Notify("CONVEX_GRID_DELTA",update_str);
    i++;
  }
}

void CoverageMap::update(bool post_update)
{

  cout << "Trying update with previous pose: ("<< m_previous_pose.mu[0] << "," << m_previous_pose.mu[1] << "," <<m_previous_pose.mu[3] << ") and current pose: (" <<
       m_current_pose.mu[0]<< "," << m_current_pose.mu[1]<< "," <<m_current_pose.mu[3] << ")" << endl;

  // step 1: is the sensor on?
  bool sensor_on = true;
  string reason = "";

  //test 1 max_change_yaw
  double dYaw = fabs(m_previous_pose.mu[3] - m_current_pose.mu[3]);
  if (dYaw > PI) dYaw = 2*PI - dYaw;
  if (dYaw > m_max_change_yaw){
    sensor_on = false;
    reason = "Change in yaw to large";
  }

  //test 2 move too big
  double dist_traveled = sqrt(pow(m_previous_pose.mu[0] - m_current_pose.mu[0],2) + 
			      pow(m_previous_pose.mu[1] - m_current_pose.mu[1],2));
  if (dist_traveled > m_max_move){
    sensor_on = false;
    reason = "Move too big. dist_traveled = " + doubleToString(dist_traveled)
      + "(" + doubleToString(m_previous_pose.mu[0]) + "," + doubleToString(m_previous_pose.mu[1]) + ")" + " (" + doubleToString(m_current_pose.mu[0]) + "," + doubleToString(m_current_pose.mu[1]) + ")";
  }

  //test 3 on surface
  if(m_current_pose.mu[2] > m_min_depth && m_block_sensor_on_surface){
    sensor_on = false;
    reason    = "Vehicle on surface";
  }

  //test 4 haven't moved at all.
  if(m_current_pose.mu[0] == m_previous_pose.mu[0] && m_current_pose.mu[1] == m_previous_pose.mu[1]){
    sensor_on = false;
    reason    = "Vehicle not moving";
  }


  if (sensor_on){ // we passed all the tests let's do the update
    vector<int> points_hit = m_coverage_map.update(m_previous_pose, m_current_pose, m_coverage_sensor);
    // use the points_hit to udpate the coverage map visual.
    if(post_update)
      updateMap(points_hit);
    m_points_updated = points_hit.size();
  }
  else
    if (reason != "Vehicle not moving")
      reportEvent("Sensor updated blocked because: " + reason);

  m_previous_pose = m_current_pose;
}
