#include "RigidTrack.h"
#include "main.h"
#include "communication.h"


RigidTrack::RigidTrack(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	
}

void RigidTrack::on_btnStopCamera_clicked()
{
	stop_camera();
}

void RigidTrack::on_btnZero_clicked()
{
	setZero();
}

void RigidTrack::on_btnCalibrate_clicked()
{
	calibrate_camera();
}

void RigidTrack::setImage(QPixmap image)
{
	ui.lbStatus->setPixmap(image);
}

void RigidTrack::clearLog()
{
	ui.listLog->reset();
}

void RigidTrack::on_btnLoadCalib_clicked()
{
	load_calibration(1);
}

void RigidTrack::setLog(QString logText)
{
	RigidTrack::ui.listLog->addItem(logText);
	RigidTrack::ui.listLog->scrollToBottom();
}

void RigidTrack::on_sbHeadingOffset_valueChanged(double d)
{
	setHeadingOffset(d);
}

void RigidTrack::on_leIPDrone_returnPressed()
{
	IPAdressDrone = QHostAddress(RigidTrack::ui.leIPDrone->text());
	commObj.addLog("Drone IP changed to:");
	commObj.addLog(RigidTrack::ui.leIPDrone->text());
}

void RigidTrack::on_leIPCB_returnPressed()
{
	IPAdressCB = QHostAddress(RigidTrack::ui.leIPCB->text());
	commObj.addLog("Safety Switch IP changed to:");
	commObj.addLog(RigidTrack::ui.leIPCB->text());
}

void RigidTrack::on_rbP3P_clicked()
{
	methodPNP = 2;
	commObj.addLog("Changed PnP algorithm to P3P");
}

void RigidTrack::on_rbIterative_clicked()
{
	methodPNP = 0;
	commObj.addLog("Changed PnP algorithm to Iterative");
}

void RigidTrack::on_actionShow_Help_triggered()
{
	show_Help();
}

void RigidTrack::on_cbSafety_stateChanged(int state)
{
	RigidTrack::ui.dsbDimension->setEnabled(state);
	RigidTrack::ui.sbAngle->setEnabled(state);
	safetyEnable = state;
	RigidTrack::ui.leIPCB->setEnabled(state);
	if (state)
	{
		commObj.addLog("Enabled Safety Area Protection");
	}
	else
	{
		commObj.addLog("Disabled Safety Area Protection");
	}
}

void RigidTrack::on_dsbDimension_valueChanged(double d)
{
	safetyBoxLength = d;
}

void RigidTrack::on_sbAngle_valueChanged(int i)
{
	safetyAngle = i;
}

void RigidTrack::on_pbLoadMarker_clicked()
{
	loadMarkerConfig(1);
}

void RigidTrack::on_btnStartCamera_clicked()
{
	if(RigidTrack::ui.btnStartCamera->text() == "Start Tracking")
	{
		RigidTrack::ui.btnStartCamera->setText("Stop Tracking");
	}
	else
	{
		RigidTrack::ui.btnStartCamera->setText("Start Tracking");
	}
	start_cameraThread();
}

