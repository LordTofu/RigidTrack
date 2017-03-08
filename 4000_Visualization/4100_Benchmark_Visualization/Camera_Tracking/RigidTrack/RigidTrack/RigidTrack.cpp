#include "RigidTrack.h"
#include "main.h"
#include "communication.h"


RigidTrack::RigidTrack(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	
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

void RigidTrack::on_leIPObject_returnPressed()
{
	QString adress = RigidTrack::ui.leIPObject->text();
	IPAdressObject = QHostAddress(adress.split(":")[0]);
	portObject = adress.split(":")[1].toInt();
	commObj.addLog("Object IP changed to:");
	commObj.addLog(IPAdressObject.toString());
	commObj.addLog("Object Port changed to:");
	commObj.addLog(QString::number(portObject));
}

void RigidTrack::on_leIPSafety_returnPressed()
{
	QString adress = RigidTrack::ui.leIPSafety->text();
	IPAdressSafety = QHostAddress(adress.split(":")[0]);
	portSafety = adress.split(":")[1].toInt();
	commObj.addLog("Safety Switch IP changed to:");
	commObj.addLog(IPAdressSafety.toString());
	commObj.addLog("Safety Switch Port changed to:");
	commObj.addLog(QString::number(portSafety));
}

void RigidTrack::on_leIPSafety2_returnPressed()
{
	QString adress = RigidTrack::ui.leIPSafety2->text();
	IPAdressSafety2 = QHostAddress(adress.split(":")[0]);
	portSafety2 = adress.split(":")[1].toInt();
	commObj.addLog("Receiver 2 IP changed to:");
	commObj.addLog(IPAdressSafety2.toString());
	commObj.addLog("Receiver 2 Port changed to:");
	commObj.addLog(QString::number(portSafety2));
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
	RigidTrack::ui.leIPSafety->setEnabled(state);
	if (state)
	{
		commObj.addLog("Enabled Safety Area Protection");
		on_leIPSafety_returnPressed();
	}
	else
	{
		commObj.addLog("Disabled Safety Area Protection");
	}
}

void RigidTrack::on_cbSafety2_stateChanged(int state)
{
	RigidTrack::ui.leIPSafety2->setEnabled(state);
	safety2Enable = state;
	if (state)
	{
		commObj.addLog("Enabled second Receiver");
		on_leIPSafety2_returnPressed();
	}
	else
	{
		commObj.addLog("Disabled second Receiver");
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

void RigidTrack::enableP3P(bool value)
{
	RigidTrack::ui.rbP3P->setEnabled(value);
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
	start_stopCamera();
}

