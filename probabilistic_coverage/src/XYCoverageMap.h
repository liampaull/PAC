/************************************************************/
/*    NAME: Liam Paull                              		*/
/*    ORGN: MIT 					*/
/*    FILE: XYCoverageMap.h                                       */
/*    DATE: November 6, 2014                                  */
/************************************************************/

#ifndef XYCoverageMapH
#define XYCoverageMapH

#include "MOOS/libMOOS/MOOSLib.h"
#include "XYConvexGrid.h"
#include "CoverageSensor.h"
#include "PDF.h"
#include <lcmtypes/coop/pose3_t.hpp> // Don't really like that this is now dependent on coop... the pose type should be defined more generally (like in libbot or isam?)
#include <string>

using namespace std;

class XYCoverageMap : public XYConvexGrid {


 public:
  XYCoverageMap();
  ~XYCoverageMap();

 public:
 vector<int> update(coop::pose3_t, coop::pose3_t, CoverageSensor,  bool real = true, int downSampleRate = 1); 
 bool initialize(const XYPolygon&, double cell_size=1, double init_val = 0.5); // overloaded
 void reset(int); 

 void set_criterion(string sval){m_completion_criterion = sval;};
 void set_completion_value(double dval){m_completion_value = dval;};
 double get_effective_range(){return m_effective_range;}; 

 private:

  PDF   RVMax(PDF, PDF);
  PDF   map_through_sensor_func(PDF, vector<double>);
  PDF   project_pose(coop::pose3_t);
  void  set_effective_range(PDF, CoverageSensor);

 private:

  std::vector<PDF> m_cell_coverage; // indexed by per cell variable (see XYConvexGrid). This is the main extension to convexgrid, now we have a coverage RV assocaited with each grid cell

  bool       m_initialized;
  double     m_completion_value;
  string     m_completion_criterion;
  double     m_effective_range;
};

#endif
