/************************************************************/
/*    NAME: Liam Paull                                		*/
/*    ORGN: UNB             					*/
/*    FILE: XYCoverageMap.cpp                               */
/*    DATE: November 16, 2011    					*/
/*             
/*    Description:                  				*/
/* Known Bugs: Don't think it's going to work correctly with grid cell size not equal to 1:
   /* the grid cell size should be the same as the discretization of the Random Vars, which should be the same as the resolution of the coverage sensor. Currently not setup to handle a discretization of the random vars other than 1. 
/************************************************************/


#include "XYCoverageMap.h"
#include <math.h>


#ifndef PI
#define PI 3.141592653589793
#endif

using namespace std;

#define DEBUG 0

XYCoverageMap::XYCoverageMap()
{
  m_initialized = false;
}


XYCoverageMap::~XYCoverageMap()
{
}

void XYCoverageMap::reset(int init_val)
{
  XYConvexGrid::reset();
  for (int i = 0; i<m_cell_coverage.size(); i++){
    m_cell_coverage[i].Reset();
    m_cell_coverage[i].AddPoint(init_val,1.0);
  }
}


//Procedure: Initialize
//Description: initialize the underlying convex grid and setup all the coverage distributions
bool XYCoverageMap::initialize(const XYPolygon& workspace,double cell_size,double init_val)
{
  XYConvexGrid::initialize(workspace,cell_size,init_val);
  for(int i = 0; i <m_elements.size(); i++){
    PDF RV;
    RV.AddPoint(init_val,1.0); //assigns all weight in the pdf to the initial value
    m_cell_coverage.push_back(RV);
  }

  m_initialized = true;
  return(true);
}


// Procedure: update. 
// Description: this is the key functin for maintaining the coverage map. Takes two poses that constitute a "move" as well as a coverage sensor. A line between the start and finish pose is parameterized and stepped along. At each point we take the pose and project it through the coverage sensor function to update the probabilities of coverage over the map. For more info see Paull, Seto and Li ICRA 2014.
vector<int> XYCoverageMap::update(coop::pose3_t startPose, coop::pose3_t endPose, CoverageSensor sensor, bool real, int downSampleRate)
{
  vector<int> pointsHit;

  // Step 1: Take the current pose and project it down to the line orthogonal to vehicle motion (yaw)

  PDF rangeRV = project_pose(endPose);
  if (DEBUG) {
    MOOSTrace("Built Range RV:\n");
    rangeRV.PrintOut();
  }

  set_effective_range(rangeRV,sensor);

  double yaw = (endPose.mu[3] + startPose.mu[3])/2; // there shouldn't be much difference between these two anyways but we take the average

  // Step 2:  paramtrize the line connecting the start and end pose. Then we will walk along that line and update at each point. This is so that we don't miss any points that the AUV hit in between updates. As a result it can update a lot less frequently 

  // step_size
  double dt;
  dt = 0.5/sqrt(pow(startPose.mu[0] - endPose.mu[0],2) + pow(startPose.mu[1] - endPose.mu[1],2)); // Changed Aug.8 2012 adaptive dt so that we take smaller steps when the poses are further apart but don't do so many unnecessary calculations.

  // we walk along the line and do an update at each point	
  for (double t = 0.0 ; t<1.0 ; t=t+dt){
    double x,y;
    x = startPose.mu[0] + t*(endPose.mu[0] - startPose.mu[0]);
    y = startPose.mu[1] + t*(endPose.mu[1] - startPose.mu[1]);

    // iterate through every point in the sensor function and do an update at the corresponding cell.
    for (double r = -sensor.range; r < sensor.range ; r+=sensor.resolution){
      // do some trig and rounding to find the point (xr, yr) which is the location at distance r from current location in direction of SSS
      double xr,yr; // the points in global frame where we will make the update
      xr = x - r*sin(yaw);
      yr = y + r*cos(yaw);

      if (m_config_poly.contains(yr,xr)){ // note conversion to MOOS coords
	if (DEBUG) MOOSTrace("XYCoverageMap: UpdateConf: Got through all the tests, doing the update.\n"); 
	// ok we have found a good point that's in the coverage area let's do the update

	int index = find_index_from_point(xr,yr); // note conversion to MOOS coords
 
	if(DEBUG) MOOSTrace("Got index %d from point (%g,%g)",index,xr,yr);

	// step 3a: take the range RV (found in step 1) and shift it to the point we are considering. This RV now represents the distribution of locataions of the cell in SENSOR frame

	if (DEBUG) MOOSTrace("ShiftMean of rangeRV:\n");
	rangeRV.ShiftMean(r); // shift mean of rangeRV 
	//if (DEBUG) rangeRV.PrintOut();				

	// step 3b: project this RV through the coverage sensor function

	if (DEBUG) MOOSTrace("Map through the PY \n");
	PDF newCoverageRV =  map_through_sensor_func(rangeRV, sensor.f);
	if (DEBUG) newCoverageRV.PrintOut(); 

	// step 3c: combine the coverage that we just calculated with the stored coverage at the point using the max operation on RVs

	if (DEBUG) MOOSTrace("Perform RVMax\n");
	PDF temp = m_cell_coverage[index];
	try{
	  m_cell_coverage[index] = RVMax(temp, newCoverageRV);
	}
	catch(char const* e){
	  cout<< e << endl;
	  throw e;
	}

	// update the cell value (for now just using mean for this but could change
	setVal(index, m_cell_coverage[index].GetMean());

	if (DEBUG) m_cell_coverage[index].PrintOut();

	rangeRV.ShiftMean(-r); // shift mean back to original for use in the next loop .. this is a little weird...

	if (pointsHit.empty()){
	  pointsHit.push_back(index);
	}
	else if (pointsHit.back() != index ){ // avoiding duplication in points hit list.
	  pointsHit.push_back(index);
	}
      }
    } // for r
  } // for t

  return pointsHit;
}


// procedure set_effective_range
// description based on the uncertainty of a pose and the given coverage sensor, how many cells would be covered if we were in an empty workspace?
// to calculate this requires knownledge of the coverage completion criterion. For now we have only implemented the probability of mean greater than P
// Do the weighted average over the coverage sensor where we weight by the probabilities in the poseRV. Basically an application of the theorem of total probability.
// Finally search through the output to find the furtherst away cell that was actually covered.
void XYCoverageMap::set_effective_range(PDF poseRV, CoverageSensor sensor)
{
  
  if (m_completion_criterion == "mean"){
    // do a convolution and then find the max range that satisfies the mission criterion
    vector<double> effective_coverage;
    vector<double> ranges;
    for (double r = 0; r<sensor.range; r+=sensor.resolution){
      double coverage=0;
      for (int i = 0; i<poseRV.numPoints; i++){
	int coverage_sensor_x = int(floor((r + poseRV.Getx(i))/sensor.resolution));
	if (coverage_sensor_x >= sensor.f.size())
	  coverage += poseRV.Getfx(i) * 0.5; //set anything out of the sensor range to be 0.5 (will avoid seg fault based on accessing out of vector range
	else{
	  coverage += poseRV.Getfx(i) * sensor.f[coverage_sensor_x];
	  //cout << "coverage_sensor.f[coverage_sensor_x] : " << sensor.f[coverage_sensor_x] << endl ;
	}
	//cout << "coverage_sensor_x: "<< coverage_sensor_x << endl;
	//cout << "poseRV.getx: " << poseRV.Getx(i) << endl;
	//cout << "poseRV.getFx: " << poseRV.Getfx(i) << endl;
      }
      effective_coverage.push_back(coverage);
      ranges.push_back(r);
    }
    //    cout << "+++++++++++++ pose RV:" << endl;
    //poseRV.PrintOut();
    //cout << "+++++++++++++ Effective Coverage vec:" << endl;
    for (int i = effective_coverage.size() - 1; i >= 0; i--){
      if (effective_coverage[i] > m_completion_value){
	m_effective_range = ranges[i];
	return;
      }
      //      cout << effective_coverage[i] << ", ";
    }
    //    cout << endl;
  }
}


PDF XYCoverageMap::RVMax(PDF RV1, PDF RV2)
{

  PDF resultRV;
	
  if (DEBUG) RV1.PrintOut();
  if (DEBUG) RV2.PrintOut();

  int i=0,j=0;
  bool RV1Complete=false, RV2Complete=false;
  int totalPoints = RV1.numPoints + RV2.numPoints;

  // This ends up being more complicated than expected. Need to add each of the points in the pdf scaled by the other cdf. If the two RVs happen to have points at the same x point, then the products need to be added together. Also need to figure out when we've hit all the points for one of the RVs so that we don't go out of range on the vector
  while (i+j < totalPoints){
    //First need to check if either one is completed
    if (RV1Complete){
      //Process point from RV2
      if (DEBUG) MOOSTrace("XYCoverageMap:RVMax: Trying to add point 1: (%f, %f)\n", RV2.Getx(j), RV2.Getfx(j)*RV1.Fx(RV2.Getx(j)));
      resultRV.AddPoint(RV2.Getx(j), RV2.Getfx(j)*RV1.Fx(RV2.Getx(j)));
      j++;
      if(j==RV2.numPoints)
	break;
      else
	continue;
    }
    if (RV2Complete){
      if (DEBUG) MOOSTrace("XYCoverageMap:RVMax: Trying to add point 2: (%f, %f)\n", RV1.Getx(i), RV1.Getfx(i)*RV2.Fx(RV1.Getx(i)));
      resultRV.AddPoint(RV1.Getx(i), RV1.Getfx(i)*RV2.Fx(RV1.Getx(i)));
      i++;
      if(i==RV1.numPoints)
	break;
      else
	continue;
    }
    // Now figure out which point is next 
    if (RV1.Getx(i) < RV2.Getx(j)) {
      //Process Point from RV1
      if (DEBUG) MOOSTrace("XYCoverageMap:RVMax: Trying to add point 3: (%f, %f)\n", RV1.Getx(i), RV1.Getfx(i)*RV2.Fx(RV1.Getx(i)));
      resultRV.AddPoint(RV1.Getx(i), RV1.Getfx(i)*RV2.Fx(RV1.Getx(i)));
      i++;
    } 
    else if (RV2.Getx(j) < RV1.Getx(i)){
      //Process Point from RV2
      if (DEBUG) MOOSTrace("XYCoverageMap:RVMax: Trying to add point 4: (%f, %f)\n", RV2.Getx(j), RV2.Getfx(j)*RV1.Fx(RV2.Getx(j)));
      resultRV.AddPoint(RV2.Getx(j), RV2.Getfx(j)*RV1.Fx(RV2.Getx(j)));
      j++;
    }
    else{
      //We have a point from both
      // Edit Aug 13, 2012. Based on my calculations there was a subtle error here. See Aug13 notes in my notebook. By applying the max formula for RVs in the straightforward way results in a double counting . Instead, you have to assume that one point is treated first and then the other. To account for this, subtract a small amount inside the Fx function of one of the RVs
      if (DEBUG) MOOSTrace("XYCoverageMap:RVMax: Trying to add point 5: (%f, %f)\n", RV1.Getx(i),  RV1.Getfx(i)*RV2.Fx(RV1.Getx(i)) + RV2.Getfx(j)*RV1.Fx(RV2.Getx(j)));
      resultRV.AddPoint(RV1.Getx(i),  RV1.Getfx(i)*RV2.Fx(RV1.Getx(i) - 0.00001) + RV2.Getfx(j)*RV1.Fx(RV2.Getx(j)));
      i++;
      j++;
    }
    if(i==RV1.numPoints)
      RV1Complete = true;
    if(j==RV2.numPoints)
      RV2Complete = true;
  }
	
  if (DEBUG) resultRV.PrintOut();	

  try{
    resultRV.Normalize();
  }
  catch(char const* E){
    cout << E;
    MOOSTrace("RV1:");
    RV1.PrintOut();
    MOOSTrace("RV2:");
    RV2.PrintOut();
    MOOSTrace("Result:");
    resultRV.PrintOut();
    throw E;
  }
  return resultRV;
}

PDF XYCoverageMap::map_through_sensor_func(PDF RV, vector<double> PYxy)
{
  int outputResolution = 100; // this represents the precision of the output.

  if (DEBUG) MOOSTrace("XYCoverageMap:map_through_sensor_func: Starting... RV.numpoints = %d \n", RV.numPoints);
  PDF confRV;
	
  double fx[outputResolution + 1];
  if (DEBUG) MOOSTrace("XYCoverageMap:map_through_sensor_func: Step 1...\n");
  // step 1: initialize everything to 0
  for (int i=0; i<outputResolution + 1; i++){
    fx[i] = 0;
  }
  // step 2: build the pdf values. Use the values of the py to calculate the indexes.
  if (DEBUG) MOOSTrace("XYCoverageMap:map_through_sensor_func: Step 2...\n");
  int index;
  for (int i=0; i<RV.numPoints; i++){
    //we have to handle the case of the Gaussian flowing outside the range of the PY
    if (fabs(RV.Getx(i)) >= int(PYxy.size()))
      index = 0; // 0 will be the bucket for 0.5
    else
      index = int(round((PYxy.at(fabs(RV.Getx(i)))-0.5)*2*outputResolution)); // abs because the PY curve is symmetrical about the vehicle position
		
    if (DEBUG) MOOSTrace(" %d ", index);
    if (index < 0 || index > outputResolution + 1) throw ("Index out of bounds Error\n");
    fx[index] += RV.Getfx(i);
  }
  if (DEBUG) MOOSTrace("XYCoverageMap:map_through_sensor_func: Step 3...\n");
  // step 3: add all the points to the pdf
  double x = 0.5;
  for (int i=0 ; i<=outputResolution; i++){
    if (fx[i] > 0){
      if (DEBUG) MOOSTrace("Trying to add point (%f,%f)\n",x,fx[i]);
      confRV.AddPoint(x,fx[i]);
    }
    x += 0.5/outputResolution;
  }
  // step 4: normalize
  if (DEBUG) MOOSTrace("XYCoverageMap:map_through_sensor_func: Step 4...\n");

  try{
    confRV.Normalize();
  }
  catch(char const* E){
    cout << E;
    confRV.PrintOut();
    RV.PrintOut();
    MOOSTrace("PY:\t");
    vector<double>::iterator p;
    for(p=PYxy.begin(); p<PYxy.end(); p++){
      MOOSTrace("%f\t",*p);
    }
    throw E;
  }

  if (DEBUG) MOOSTrace("XYCoverageMap:map_through_sensor_func: Finishing\n");
  return confRV;

}
	

PDF XYCoverageMap::project_pose(coop::pose3_t pose)
{
  double rho; 
  double sXX = pose.sigma[0][0];
  double sYY = pose.sigma[1][1];
  double sXY = pose.sigma[0][1];
  double yaw = pose.mu[3];

  rho = sXY/(sqrt(sXX)*sqrt(sYY)); 
  if (rho < 0){
    if (DEBUG) MOOSTrace("XYCoverageMap:pose_projection: Warning! rho < 0, setting to 0\n");
    rho = 0;
  }
  double Sigma = sqrt(sXX*pow(sin(yaw),2) + 2*rho*sqrt(sXX)*sqrt(sYY)*sin(yaw)*cos(yaw) + sYY*pow(cos(yaw),2));
  if (DEBUG) MOOSTrace("XYCoverageMap:pose_projection: Sigma = %f\n", Sigma);
  PDF rangeRV;
  rangeRV.Gaussian(0,Sigma);

  return rangeRV;

}

			

