#include "OcuGuiWidget.h"
#include <iostream>

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  OcuGuiWidget window;
  //window.resize(400, 600);
  window.setWindowTitle("Ocu GUI");
  window.show();
  
  return app.exec();
}


OcuGuiWidget::OcuGuiWidget(QWidget *parent)
    : QWidget(parent)
{   
  // Create Quit button
  QPushButton *quit = new QPushButton("&Quit");
  connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));

  // Create OCU
  createOCUGroupBox();

  // Create CCC
  createCCCGroupBox();

  // Create PP
  createPPGroupBox();

  // Create Sensors
  createSensorGroupBox();

  // Create Status
  createStatusGroupBox();

  //// Create timer
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateStatus()) );
  timer->start(50); 

  //// Create main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(ocuGroupBox);
  mainLayout->addWidget(cccGroupBox);
  mainLayout->addWidget(ppGroupBox);
  mainLayout->addWidget(sensorGroupBox);
  mainLayout->addWidget(statusGroupBox);
  mainLayout->addWidget(quit);
  this->setLayout(mainLayout);
}


void OcuGuiWidget::updateStatus()
{
  //std::cout << "Updating status..." << std::endl;
  client.send();

  client.receive();
  ocuStatus->setText( QString( client.receive_status.c_str() ) );

  rseStatus->setText( QString( client.rse_status.c_str() ) );
  cccStatus->setText( QString( client.ccc_status.c_str() ) );
  ppStatus->setText( QString( client.pp_status.c_str() ) );
  monitorStatus->setText( QString( client.monitor_status.c_str() ) );

  fsmStatus->setText( QString( client.fsm_status.c_str() ) );
  fsmActuator->setText( QString( client.fsm_actuator.c_str() ) );
  baseStatus->setText( QString( client.base_status.c_str() ) );
  actuatorStatus->setText( QString( client.actuator_status.c_str() ) );
  rseStatus->setText( QString( client.rse_status.c_str() ) );

  std::stringstream front;
  front << "Front Magnetometer: " << (int) client.mag_front;
  std::stringstream rear;
  rear << "Rear Magnetometer: " << (int) client.mag_rear;
  gpsValue->setText( QString( client.gps.c_str() ) );
  magFront->setValue( client.mag_front );
  magFrontText->setText( QString( front.str().c_str() ) );
  magRear->setValue( client.mag_rear );
  magRearText->setText( QString( rear.str().c_str() ) );
}

void OcuGuiWidget::createCCCGroupBox()
{
  cccGroupBox = new QGroupBox(tr("CCC"));
  QVBoxLayout *layout = new QVBoxLayout;
  QHBoxLayout *layout1 = new QHBoxLayout;

  QLabel *cccSpeedLabel = new QLabel( tr("CCC target speed: " ) );
  layout1->addWidget(cccSpeedLabel);

  cccSpeed = new QDoubleSpinBox;
  cccSpeed->setRange(0, 2.0);
  cccSpeed->setSingleStep(0.1);
  cccSpeed->setValue(0.0);
  layout1->addWidget(cccSpeed);
  connect( cccSpeed, SIGNAL( valueChanged(double) ), this, SLOT(updateCCCSpeed(double)) );

  cccStartButton = new QPushButton(tr("Start CCC"));
  layout1->addWidget(cccStartButton);
  connect( cccStartButton, SIGNAL(clicked()), this, SLOT(startCCC()));
  cccStopButton = new QPushButton(tr("Stop CCC"));
  layout1->addWidget(cccStopButton);
  connect( cccStopButton, SIGNAL(clicked()), this, SLOT(stopCCC()));
  layout->addLayout( layout1 );

  rseStatus = new QLabel( tr("RSE: " ) );
  layout->addWidget(rseStatus);
  cccStatus = new QLabel( tr("CCC: " ) );
  layout->addWidget(cccStatus);

  cccGroupBox->setLayout(layout);
}

void OcuGuiWidget::createPPGroupBox()
{
  ppGroupBox = new QGroupBox(tr("PP"));
  QVBoxLayout *layout = new QVBoxLayout;

  QHBoxLayout *layout1 = new QHBoxLayout;
  QPushButton *loadMapButton = new QPushButton( tr("Load map" ) );
  layout1->addWidget(loadMapButton);
  connect( loadMapButton, SIGNAL(clicked()), this, SLOT(loadMap()));
  mapFileLabel = new QLabel( tr("(none)" ) );
  layout1->addWidget(mapFileLabel);
  //layout->addLayout( layout1 );

  //QHBoxLayout *layout2 = new QHBoxLayout;
  QLabel *goalLatLabel = new QLabel( tr("Latitude: " ) );
  layout1->addWidget(goalLatLabel);
  ppLat = new QDoubleSpinBox;
  ppLat->setRange(-90, 90);
  ppLat->setSingleStep(0.00001);
  ppLat->setValue(0.0);
  ppLat->setDecimals(5);
  layout1->addWidget(ppLat);
  connect( ppLat, SIGNAL( valueChanged(double) ), this, SLOT(updatePPLat(double)) );
  layout->addLayout( layout1 );

  QHBoxLayout *layout3 = new QHBoxLayout;
  QLabel *goalLonLabel = new QLabel( tr("Longitude: " ) );
  layout3->addWidget(goalLonLabel);
  ppLon = new QDoubleSpinBox;
  ppLon->setRange(-180, 180.0);
  ppLon->setSingleStep(0.00001);
  ppLon->setValue(0.0);
  ppLon->setDecimals(5);
  layout3->addWidget(ppLon);
  connect( ppLon, SIGNAL( valueChanged(double) ), this, SLOT(updatePPLon(double)) );
  //layout->addLayout( layout3 );

  //QHBoxLayout *layout4 = new QHBoxLayout;
  QLabel *goalAltLabel = new QLabel( tr("Altitude: " ) );
  layout3->addWidget(goalAltLabel);
  ppAlt = new QDoubleSpinBox;
  ppAlt->setRange(-10000, 10000.0);
  ppAlt->setSingleStep(0.1);
  ppAlt->setValue(0.0);
  ppAlt->setDecimals(1);
  layout3->addWidget(ppAlt);
  connect( ppAlt, SIGNAL( valueChanged(double) ), this, SLOT(updatePPAlt(double)) );
  layout->addLayout( layout3 );

  QHBoxLayout *layout5 = new QHBoxLayout;
  ppStartButton = new QPushButton(tr("Start PP"));
  layout5->addWidget(ppStartButton);
  connect( ppStartButton, SIGNAL(clicked()), this, SLOT(startPP()));
  ppStopButton = new QPushButton(tr("Stop PP"));
  layout5->addWidget(ppStopButton);
  connect( ppStopButton, SIGNAL(clicked()), this, SLOT(stopPP()));
  layout->addLayout( layout5 );

  QHBoxLayout *layout6 = new QHBoxLayout;
  ppStatus = new QLabel( tr("PP: " ) );
  layout6->addWidget(ppStatus);

  monitorStatus = new QLabel( tr("Monitor: " ) );
  layout6->addWidget(monitorStatus);
  layout->addLayout( layout6 );

  ppGroupBox->setLayout(layout);
}

void OcuGuiWidget::createOCUGroupBox()
{
  ocuGroupBox = new QGroupBox(tr("OCU Server connection"));
  QVBoxLayout *layout = new QVBoxLayout;

  ocuStatus = new QLabel( tr("OCU: " ) );
  layout->addWidget(ocuStatus);

  ocuGroupBox->setLayout(layout);
}

void OcuGuiWidget::createStatusGroupBox()
{
  statusGroupBox = new QGroupBox(tr("Status"));
  QVBoxLayout *layout = new QVBoxLayout;

  fsmStatus = new QLabel( tr("FSM: " ) );
  layout->addWidget(fsmStatus);
  fsmActuator = new QLabel( tr("FSM Actuator: " ) );
  layout->addWidget(fsmActuator);
  baseStatus = new QLabel( tr("Base: " ) );
  layout->addWidget(baseStatus);
  actuatorStatus = new QLabel( tr("Actuator: " ) );
  layout->addWidget(actuatorStatus);

  statusGroupBox->setLayout(layout);
}

void OcuGuiWidget::createSensorGroupBox()
{
  QLabel *N = new QLabel( tr(" N " ) );
  QLabel *S = new QLabel( tr(" S " ) );
  QLabel *N2 = new QLabel( tr(" N " ) );
  QLabel *S2 = new QLabel( tr(" S " ) );
  sensorGroupBox = new QGroupBox(tr("Sensors"));
  QVBoxLayout *layout = new QVBoxLayout;

  gpsValue = new QLabel( tr("GPS: " ) );
  layout->addWidget(gpsValue);

  QHBoxLayout *layout1 = new QHBoxLayout;
  QVBoxLayout *vlayout1 = new QVBoxLayout;
  QVBoxLayout *vlayout2 = new QVBoxLayout;
  magFrontText = new QLabel( tr( "Front Magnetometer" ) );
  magFront = new QDial( );
  magFront->setRange( 0, 359 );
  magFront->setNotchTarget( 30 );
  magFront->setWrapping( true );
  magFront->setNotchesVisible( true );
  vlayout1->addWidget(N, 0, Qt::AlignCenter);
  vlayout1->addWidget(magFront, 0, Qt::AlignCenter);
  vlayout1->addWidget(S, 0, Qt::AlignCenter);
  vlayout1->addWidget(magFrontText, 0, Qt::AlignCenter);
  magRearText = new QLabel( tr( "Rear Magnetometer" ) );
  magRear = new QDial( );
  magRear->setRange( 0, 359 );
  magRear->setNotchTarget( 30 );
  magRear->setWrapping( true );
  magRear->setNotchesVisible( true );
  vlayout2->addWidget(N2, 0, Qt::AlignCenter);
  vlayout2->addWidget(magRear, 0, Qt::AlignCenter);
  vlayout2->addWidget(S2, 0, Qt::AlignCenter);
  vlayout2->addWidget(magRearText, 0, Qt::AlignCenter);
  layout1->addLayout( vlayout1 );
  layout1->addLayout( vlayout2 );

  layout->addLayout( layout1 );

  sensorGroupBox->setLayout(layout);
}


void OcuGuiWidget::loadMap( ) 
{
  QString filename = QFileDialog::getOpenFileName( this, tr("Open File..."),
      QString(), tr("CMU map (*.map);;All Files (*)"));

  {
    QString dots( "..." );
    size_t maxlen( 16 );
    if ( filename.size() > maxlen + dots.size() ) {
      int n1 = filename.size() - maxlen - dots.size();
      filename.replace( 0, n1, dots.data(), dots.size() );
    }
  }
  mapFileLabel->setText( filename );
  client.pp_map_file = filename.toUtf8().constData();
  std::cout << "Map file: " << client.pp_map_file << std::endl;
}
