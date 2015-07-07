#pragma once

#include <QApplication>
#include <QWidget>
#include <QTimer>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QFileDialog>
#include <QDial>

#include "ocu_client.h"

class OcuGuiWidget : public QWidget {
  Q_OBJECT 

  public:
    OcuGuiWidget( QWidget *parent = 0 );

  private slots:
    void updateStatus();

    inline void updateCCCSpeed( double value ) {
      std::cout << "Updated CCC speed: " << value << std::endl;
      client.ccc_speed = value;
    }

    inline void startCCC() {
      std::cout << "Starting CCC" << std::endl;
      client.ccc_start = true;
    }

    inline void stopCCC() {
      std::cout << "Stopping CCC" << std::endl;
      client.ccc_stop = true;
    }

    // PP slots
    void loadMap( );

    inline void updatePPLat( double value ) {
      std::cout << "Updated PP Lat: " << value << std::endl;
      client.pp_goal_lat = value;
    }

    inline void updatePPLon( double value ) {
      std::cout << "Updated PP Lon: " << value << std::endl;
      client.pp_goal_lon = value;
    }

    inline void updatePPAlt( double value ) {
      std::cout << "Updated PP Alt: " << value << std::endl;
      client.pp_goal_alt = value;
    }

    inline void startPP() {
      std::cout << "Starting PP" << std::endl;
      client.pp_start = true;
    }

    inline void stopPP() {
      std::cout << "Stopping PP" << std::endl;
      client.pp_stop = true;
    }

  private:
    QTimer *timer;
    ocu_client client;

    QGroupBox *cccGroupBox;
    QPushButton *cccStartButton;
    QPushButton *cccStopButton;
    QDoubleSpinBox *cccSpeed;
    QLabel *rseStatus;
    QLabel *cccStatus;
    QLabel *ppStatus;
    QLabel *monitorStatus;

    QGroupBox *ppGroupBox;
    QPushButton *ppStartButton;
    QPushButton *ppStopButton;
    QLabel *mapFileLabel;
    QDoubleSpinBox *ppLat;
    QDoubleSpinBox *ppLon;
    QDoubleSpinBox *ppAlt;

    QGroupBox *statusGroupBox;
    QLabel *fsmStatus;
    QLabel *fsmActuator;
    QLabel *baseStatus;
    QLabel *actuatorStatus;

    QGroupBox *ocuGroupBox;
    QLabel *ocuStatus;

    QGroupBox *sensorGroupBox;
    QLabel *gpsValue;
    QDial *magFront;
    QDial *magRear;
    QLabel *magFrontText;
    QLabel *magRearText;

    void createCCCGroupBox();
    void createPPGroupBox();
    void createStatusGroupBox();
    void createSensorGroupBox();
    void createOCUGroupBox();
};

