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
 * \file    multi_window_anomaly_monitor.cpp
 * \brief   Detects unmodeled failures using multi scale time windows
 * \author  Juan Pablo Mendoza
 */
//========================================================================

#include "multi_window_anomaly_monitor.h"
#include <stdio.h>
#include <boost/math/distributions/normal.hpp>
#include <math.h>
#include "cmu_z_test_monitor/AnomalyMsg.h"

pthread_mutex_t AnomalyMonitor::monitorMainMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t AnomalyMonitor::monitorTmpMutex = PTHREAD_MUTEX_INITIALIZER;

AnomalyMonitor::AnomalyMonitor() {
}

AnomalyMonitor::~AnomalyMonitor() {
}

void AnomalyMonitor::init(ros::NodeHandle* n,
                          bool use_sample_var){
  bool debug = false;
  if(debug) printf("initializing cmu_z_test_monitor\n");
  useSampleVariance = use_sample_var;
  if(cdfTable.size()==0){//cdfTable has not been initialized yet
      boost::math::normal normDist;
    for (double j=CDF_MIN;j<=CDF_MAX;j+=CDF_STEP_SIZE){
      cdfTable.push_back(boost::math::cdf(normDist,j));
    }
  }
  maxOverallAnomaly = -1;
  maxAnomaly = -1;
  anomalies.resize(nScales+1);
  anomalyStats.resize(nScales+1);
  for (int i = 0;i<nScales+1;i++) anomalies[i] = -1;
  anomalyPublisher=n->advertise<cmu_z_test_monitor::AnomalyMsg>(
      "/landshark/cmu_z_test_monitor/AnomalyMonitor",1);

  double deviation;
  n->param("/landshark/cmu_z_test_monitor/max_anomaly", anomaly_thresh,0.9);
  n->param("/landshark/cmu_z_test_monitor/obs_variance", variance,0.001);
  n->param("/landshark/cmu_z_test_monitor/acceptable_deviation", deviation,0.001);
  acceptableDeviation.push_back(deviation);

  if(debug) printf("cmu_z_test_monitor initialized\n");
}

void AnomalyMonitor::calculateMaxAnomaly() {
  bool debug = false;
  pthread_mutex_lock(&monitorMainMutex);
  maxAnomaly = -1;
  for (int i = 0;i<nScales+1;i++){//+1 for global anomaly
    if(data.size()>0){
      anomalies[i] = calculateAnomaly(anomalyStats[i]);
      if(anomalies[i]>maxAnomaly){
        maxAnomaly = anomalies[i];
        maxAnomalyStat = anomalyStats[i];
      }
    }
  }
  if(maxAnomaly>maxOverallAnomaly){
    maxOverallAnomaly = maxAnomaly;
    maxOverallStat = maxAnomalyStat;
    maxOverallState = currState;
  }
  if(debug){
    printf("anomalies:\n");
    for(int i=0;i<nScales;i++){
      printf("window size: %u, anomaly: %f",WINDOW_SCALES[i],anomalies[i]);
      if(anomalyStats[i].n>0)
        printf(" stat mean[0]:%f, stat n: %f",anomalyStats[i].mean[0],
            anomalyStats[i].n);
        printf("\n");
    }
    printf("global anomaly: %f",anomalies[nScales]);
    if(anomalyStats[nScales].n>0)
      printf(" stat mean[0]:%f, stat n: %f\n",anomalyStats[nScales].mean[0],
          anomalyStats[nScales].n);
  }
  pthread_mutex_unlock(&monitorMainMutex);
}

void AnomalyMonitor::reset() {
  pthread_mutex_lock(&monitorMainMutex);
  pthread_mutex_lock(&monitorTmpMutex);
  data.clear();
  data_tmp.clear();
  for(unsigned int i=0;i<anomalies.size();i++)
    anomalies[i]=-1.0;
  for(unsigned int i=0;i<anomalyStats.size();i++)
    anomalyStats[i].n = 0.0;
  maxAnomaly = 0.0;
  maxAnomalyStat.n = 0.0;
  pthread_mutex_unlock(&monitorTmpMutex);
  pthread_mutex_unlock(&monitorMainMutex);
}

void AnomalyMonitor::addDataPoint(state s, stateStat v){
  bool debug = false;
  pthread_mutex_lock(&monitorTmpMutex);
  if(debug){
    printf("adding new data point:\nstate: ");s.print();printf("\n");
    printf("stat: n=%4.3f, mean=%4.3f, ex2=%4.3f, var=%4.3f\n",
         v.n,v.mean[0],v.ex2[0],v.variance[0]);
  }
  data_tmp.push_front(std::make_pair(s,v));
  pthread_mutex_unlock(&monitorTmpMutex);
}

void AnomalyMonitor::flushNewData(){
  bool debug = false;
  pthread_mutex_lock(&monitorMainMutex);
  pthread_mutex_lock(&monitorTmpMutex);

  if (debug) printf("flushing data:\n");
  if (debug){
    for(unsigned int i=0;i<data_tmp.size();i++){
      printf("data_tmp[%d] = ",i);data_tmp[i].first.print();
      printf(" value: %f\n",data_tmp[i].second.mean[0]);
    }
    for(unsigned int i=0;i<data.size();i++){
      printf("data[%d] = ",i);data[i].first.print();
      printf(" value: %f\n",data[i].second.mean[0]);
    }
  }
  //move elements from data_tmp to data
  while(!data_tmp.empty()){
    pair<state,stateStat> toAdd = data_tmp.back();
    data_tmp.pop_back();
    data.push_front(toAdd);
    currState = toAdd.first;
    //update the statistics for each of the windows of different scales
    //i.e., for a window of size ws add the new data point, and remove excess.
    for(int i = 0;i<nScales;i++){
      anomalyStats[i] = joinStats(anomalyStats[i],toAdd.second);
      unsigned int ws = WINDOW_SCALES[i];
      if (data.size()>ws){//must remove oldest element of this window
        pair<state,stateStat> toRemove = data[ws];
        anomalyStats[i] = removeStats(anomalyStats[i],toRemove.second);
      }
    }
    //update the last element of anomalyStats: all-encompassing window
    anomalyStats[nScales] = joinStats(anomalyStats[nScales],toAdd.second);
    //if maximum capacity has been reached, forget the last datapoint
    if(data.size()>WINDOW_SCALES[nScales-1])
      data.pop_back();
  }  
  data_tmp.clear();
  pthread_mutex_unlock(&monitorTmpMutex);
  pthread_mutex_unlock(&monitorMainMutex);
}

AnomalyMonitor::stateStat AnomalyMonitor::joinStats(stateStat stat1,
                                                    stateStat stat2){
  bool debug = false;
  if(stat1.n==0.0 || stat1.n==-0.0) return stat2;
  if(stat2.n==0.0 || stat2.n==-0.0) return stat1;
  if(debug) printf("stat1 n:%f, mean size:%lu\n",stat1.n,stat1.mean.size());
  if(debug) printf("stat2 n:%f, mean size:%lu\n",stat2.n,stat2.mean.size());
  stateStat newStat;
  unsigned int d = stat1.mean.size();
  newStat.n = stat1.n+stat2.n;
  //join means
  newStat.mean.resize(d);
  for(unsigned int i=0;i<d;i++)
    newStat.mean[i] = (stat1.mean[i]*stat1.n+stat2.mean[i]*stat2.n)/newStat.n;
  //join ex2s
  newStat.ex2.resize(d);
  for(unsigned int i=0;i<d;i++)
    newStat.ex2[i] = (stat1.ex2[i]*stat1.n+stat2.ex2[i]*stat2.n)/newStat.n;
  //join variances (either sample or given)
  newStat.variance.resize(d);
  if (useSampleVariance)
    for(unsigned int i=0;i<d;i++)
      newStat.variance[i] = newStat.ex2[i] - (newStat.mean[i]*newStat.mean[i]);
  else //using a priori known variances
    for(unsigned int i=0;i<d;i++)
      newStat.variance[i] = (stat1.variance[i]*stat1.n+
          stat2.variance[i]*stat2.n)/newStat.n;
  return newStat;
}

AnomalyMonitor::stateStat AnomalyMonitor::removeStats(stateStat stat1,
                                                      stateStat stat2){
  stateStat resultStat;
  //if stat2 contains at least as many datapoints as stat1, return empty stat
  if(stat2.n>=stat1.n){
    resultStat.n=0;
  }
  //otherwise removing stat2 is equivalent to adding stat2 with -n
  else {
    stat2.n = -stat2.n;
    resultStat = joinStats(stat1,stat2);
  }
  return resultStat;
}


double AnomalyMonitor::calculateAnomaly(AnomalyMonitor::stateStat stat)
{
  double z;//standardized version of stat.mean
  double minP=1.0;//p value. How likely is the data given the normal model?
  for (unsigned int i=0;i<stat.mean.size();i++){
    double p;
    if (stat.mean[i]>acceptableDeviation[i]){
      z = (stat.mean[i] - acceptableDeviation[i])/sqrt(stat.variance[i]/stat.n);
      p = (1-getStdNormCDF(z))*2; //*2 if two-tailed p value, *1 otherwise
    }
    else if (stat.mean[i]<-acceptableDeviation[i]){
      z = (stat.mean[i]-(-acceptableDeviation[i]))/sqrt(
          stat.variance[i]/stat.n);
      p = getStdNormCDF(z)*2;//*2 if two-tailed p value, *1 otherwise
    }
    else {//stat.mean is within the acceptable deviation
      p = 1.0;
    }
    if(p<minP) minP = p;
  }
  return (1-minP);//1 - probability of data given the normal model
}

double AnomalyMonitor::getStdNormCDF(double x) {
  if(x<=CDF_MIN || x>=CDF_MAX)
    return 0.0;//too small to care
  int xInd = floor((x-CDF_MIN)/CDF_STEP_SIZE);
  return cdfTable[xInd];
}

void AnomalyMonitor::run() {
  flushNewData();
  calculateMaxAnomaly();
  cmu_z_test_monitor::AnomalyMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.anomaly_value = maxAnomaly;
  msg.anomaly_detected = maxAnomaly > anomaly_thresh;
  anomalyPublisher.publish(msg);
}
