/*!
* @file RigidTrack.cpp
*  @brief     Rigid Track GUI source that contains functions for GUI events.
*  @author    Florian J.T. Wachter
*  @version   1.0
*  @date      April, 8th 2017
*/

#include "RigidTrack.h"
#include <QProcess>
#include <QdesktopServices>
#include <QDir>
#include <QMessageBox>
#include <QUrl>
#include "main.h"
#include "communication.h"
#include <exception>	


RigidTrack::RigidTrack(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	
}

void RigidTrack::on_btnZero_clicked()
{
	setReference();
}

void RigidTrack::on_btnCalibrate_clicked()
{
	calibrateCamera();
}

void RigidTrack::setImage(QPixmap image)
{
	ui.lbStatus->setPixmap(image);
}

void RigidTrack::clearLog()
{
	ui.listLog->reset();
}

void RigidTrack::progressUpdate(int value)
{
	RigidTrack::ui.progressBar->setValue(value);
}

void RigidTrack::on_btnLoadCalib_clicked()
{
	loadCalibration(1);
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
	try{
	QString adress = RigidTrack::ui.leIPObject->text();
	IPAdressObject = QHostAddress(adress.split(":")[0]);
	if (IPAdressObject.isNull() || adress.split(":").length() == 1 || adress.split(":")[1]==0)
	{
		throw 2;
	}
	portObject = adress.split(":")[1].toInt();
	commObj.addLog("Object IP changed to:");
	commObj.addLog(IPAdressObject.toString());
	commObj.addLog("Object Port changed to:");
	commObj.addLog(QString::number(portObject));
	}
	catch (...)
	{
		commObj.addLog("Error Changing the IP Adress or Port! Restored Standard Values 192.168.0.1:9155");
		IPAdressObject = QHostAddress("192.168.0.1");
		portObject = 9155;
		RigidTrack::ui.leIPObject->setText("192.168.0.1:9155");
	}
}


void RigidTrack::on_leIPSafety_returnPressed()
{
	try{
	QString adress = RigidTrack::ui.leIPSafety->text();
	IPAdressSafety = QHostAddress(adress.split(":")[0]);
	if (IPAdressSafety.isNull() || adress.split(":").length() == 1 || adress.split(":")[1] == 0)
	{
		throw 2;
	}
	portSafety = adress.split(":")[1].toInt();
	commObj.addLog("Safety Switch IP changed to:");
	commObj.addLog(IPAdressSafety.toString());
	commObj.addLog("Safety Switch Port changed to:");
	commObj.addLog(QString::number(portSafety));
	}
	catch (...)
	{
		commObj.addLog("Error Changing the IP Adress or Port! Restored Standard Values 192.168.0.1:9155");
		IPAdressSafety = QHostAddress("192.168.0.1");
		portSafety = 9155;
		RigidTrack::ui.leIPSafety->setText("192.168.0.1:9155");
	}
	}

void RigidTrack::on_leIPSafety2_returnPressed()
{
	try {
	QString adress = RigidTrack::ui.leIPSafety2->text();
	IPAdressSafety2 = QHostAddress(adress.split(":")[0]);
	if (IPAdressSafety2.isNull() || adress.split(":").length() == 1 || adress.split(":")[1] == 0)
	{
		throw 2;
	}
	portSafety2 = adress.split(":")[1].toInt();
	commObj.addLog("Receiver 2 IP changed to:");
	commObj.addLog(IPAdressSafety2.toString());
	commObj.addLog("Receiver 2 Port changed to:");
	commObj.addLog(QString::number(portSafety2));
	}
	catch (...)
	{
		commObj.addLog("Error Changing the IP Adress or Port! Restored Standard Values 192.168.0.1:9155");
		IPAdressSafety2 = QHostAddress("192.168.0.1");
		portSafety2 = 9155;
		RigidTrack::ui.leIPSafety2->setText("192.168.0.1:9155");
	}
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

void RigidTrack::on_rbEPnP_clicked()
{
	methodPNP = 1;
	commObj.addLog("Changed PnP algorithm to EPnP");
}

void RigidTrack::on_actionShow_Help_triggered()
{
	
	//!/< append help.pdf to the path since this is the documentation in html format
	QString qtStrFile = QDir::currentPath().replace("/", "\\") + "\\help.pdf";

	//!/< open the documentation help file in the standard browser
	QDesktopServices::openUrl(QUrl::fromLocalFile(qtStrFile));
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

void RigidTrack::on_cbInvert_stateChanged(int state)
{
	if (state)
	{
		invertZ = -1;
	}
	else
	{
		invertZ = 1;
	}
}

void RigidTrack::enableP3P(bool value)
{
	RigidTrack::ui.rbP3P->setEnabled(value);
}

void RigidTrack::on_btnCalibrateGround_clicked()
{
	calibrateGround();
}

void RigidTrack::on_actionOpen_Log_Folder_triggered()
{
	QString command = "explorer.exe " + QDir::currentPath().replace("/", "\\") + "\\logs";
	QProcess::startDetached(command);
}

void RigidTrack::on_actionAbout_Rigid_Track_triggered()
{
	QMessageBox msgBox;
	msgBox.setWindowTitle("About Rigid Track");
	msgBox.setText("Rigid Track\nInstitute for Flight System Dynamics\nVersion:\t 1.0\nAuthor:\t Florian Wachter\nBuild Date:\t " + QString(__DATE__));
	msgBox.exec();
}

void RigidTrack::on_actionOpen_Installation_Folder_triggered()
{
	QString command = "explorer.exe " + QDir::currentPath().replace("/", "\\");
	QProcess::startDetached(command);
	
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
	startStopCamera();
}

