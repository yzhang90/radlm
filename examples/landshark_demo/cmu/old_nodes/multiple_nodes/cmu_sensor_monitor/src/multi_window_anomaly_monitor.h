//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
 * \file    multi_window_anomaly_monitor.h
 * \brief   Detects unmodeled failures using multi scale time windows
 * \author  Juan Pablo Mendoza
 */
//========================================================================

#include <set>
#include <map>
#include <vector>
#include <deque>
#include <pthread.h>
#include <ros/ros.h>

#ifndef ANOMALY_MONITOR
#define ANOMALY_MONITOR
using std::vector;
using std::pair;
using std::deque;
using std::set;

//constants for generation of a standard normal CDF lookup table
static const double CDF_STEP_SIZE = 0.01;
static const double CDF_MIN = -8.0;
static const double CDF_MAX = 8.0;
static vector<double> cdfTable;
//sizes of windows of data in which anomalies will be searched for
static const unsigned int WINDOW_SCALES[] = {1,2,4,8,16,32,64,128,256,512,
    1024,2048,4096,8192,16384};
static const int nScales = sizeof(WINDOW_SCALES)/sizeof(int);

/*
 * Detects anomalies in execution by comparing data received in different
 * time windows to the expected value of such data as defined by some
 * child class that provides a model of normal behavior.
 */
class AnomalyMonitor{
public:
  //state of the robot. Just a wrapper for a vector of doubles
  struct state {
    vector<double> s;
    bool operator== (state param) const{
      return s==param.s;
    }
    bool operator< (const state param) const{
      return s<param.s;
    }
    void print() const {
      printf("[");
      for(unsigned int i=0;i<s.size();i++)
        printf("%f,",s[i]);
      printf("]");
    }
  };
  //statistics about data observed in a particular state or set of states.
  struct stateStat {
    vector<double> mean;//sample mean
    vector<double> ex2;//E[x^2] (for variance)
    double n;//number of samples (double because it can be interpolated)
    vector<double> variance;//known variance (if using sample, then use ex2 to calculate)
  };
protected:
  //avoid conflicts in shared resources (data)
  static pthread_mutex_t monitorMainMutex;
  //avoid conflicts in temporary data structure
  static pthread_mutex_t monitorTmpMutex;
  
  
  //true: use sample variance, false: use known initialized variance
  bool useSampleVariance;
  //acceptable residual deviation from 0.0 in the long run
  vector<double> acceptableDeviation;
  //<state,value> pairs from visited states
  deque<pair<state,stateStat> > data;
  //temporary storage for data when active mutex
  deque<pair<state,stateStat> > data_tmp;
  //current state of the robot
  state currState; 
  //calculated anomaly for each window size, plus one for whole data
  vector<double> anomalies;
  //anomaly statistics for each window size
  vector<stateStat> anomalyStats;
  //maximum anomaly out of different window anomalies
  double maxAnomaly;
  //statistics from which maximum anomaly is derived
  stateStat maxAnomalyStat;
  //maximum anomaly during this execution
  double maxOverallAnomaly;
  //state in which maxOverallAnomaly was discovered
  state maxOverallState;
  //stat from which maxOverallAnomaly was derived
  stateStat maxOverallStat;
  //set of states that are truly anomalous
  set<state> trueAnomalyStates;
  //stats for the set trueAnomalyStates
  stateStat trueAnomalyStat;
  
  //variance passed in as a parameter
  double variance;
  //anomaly threshold passed in as a paramter
  double anomaly_thresh;

  
  ros::Publisher anomalyPublisher;


  //insert new state,stat value into the monitor
  void addDataPoint(state s,stateStat v);

  //flush new data from addDataPoint calls once mutexes are good
  void flushNewData();
  
  //calculate the maximum anomaly value and region. Core of the algorithm
  void calculateMaxAnomaly();
  
  //calculate the anomaly of a particular region based on its stats.
  double calculateAnomaly(stateStat v);

  //groups two statistics together
  stateStat joinStats(stateStat stat1, stateStat stat2);
  //removes statistic stat2 from statistic stat1
  stateStat removeStats(stateStat stat1, stateStat stat2);
  //returns an approximate value of the standard normal CDF at x
  double getStdNormCDF(double x);

  
public:
  AnomalyMonitor();
  ~AnomalyMonitor();
  //get the anomaly value currently stored
  double getAnomaly(){return maxAnomaly;};
  //forget all the data
  void reset();
  //update anomaly and publish it
  void run();
  void init(ros::NodeHandle* n, bool use_sample_var=false);
  char resultS [200];
};

#endif
