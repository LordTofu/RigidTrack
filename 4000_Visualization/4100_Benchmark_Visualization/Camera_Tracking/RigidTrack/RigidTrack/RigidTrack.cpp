#include "RigidTrack.h"
#include "main.h"



RigidTrack::RigidTrack(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	
}

void RigidTrack::setStatus(QString status_text)
{
	RigidTrack::ui.lbLog->setText(status_text);
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

void RigidTrack::on_btnStartCamera_clicked()
{
	RigidTrack::ui.btnStartCamera->setText("Camera Started");
	int error = 0;
	error = start_camera();
}

