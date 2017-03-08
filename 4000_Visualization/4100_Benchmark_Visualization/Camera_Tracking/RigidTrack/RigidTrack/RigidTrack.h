#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_RigidTrack.h"
#include <qpixmap.h>
#include "main.h"
#include "communication.h"

class RigidTrack : public QMainWindow
{
    Q_OBJECT

public:
    RigidTrack(QWidget *parent = Q_NULLPTR);

public slots:
	
	void on_btnStartCamera_clicked();
	void on_btnZero_clicked();
	void on_btnCalibrate_clicked();
	void setImage(QPixmap image);
	void clearLog();
	void on_btnLoadCalib_clicked();
	void setLog(QString logText);
	void on_sbHeadingOffset_valueChanged(double d);
	void on_leIPObject_returnPressed();
	void on_leIPSafety_returnPressed();
	void on_leIPSafety2_returnPressed();
	void on_rbP3P_clicked();
	void on_rbIterative_clicked();
	void on_actionShow_Help_triggered();
	void on_cbSafety_stateChanged(int state);
	void on_cbSafety2_stateChanged(int state);
	void on_dsbDimension_valueChanged(double d);
	void on_sbAngle_valueChanged(int i);
	void on_pbLoadMarker_clicked();
	void enableP3P(bool value);

private:
	Ui::RigidTrackClass ui;
    
};
