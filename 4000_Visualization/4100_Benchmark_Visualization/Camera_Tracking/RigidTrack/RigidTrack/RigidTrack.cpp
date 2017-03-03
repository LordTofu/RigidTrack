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

void RigidTrack::on_btnLoadCalib_clicked()
{
	load_calibration();
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
	change_IPAddress(RigidTrack::ui.leIPDrone->text());
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

void RigidTrack::on_btnStartCamera_clicked()
{
	RigidTrack::ui.btnStartCamera->setText("Camera Started");
	start_cameraThread();
}

