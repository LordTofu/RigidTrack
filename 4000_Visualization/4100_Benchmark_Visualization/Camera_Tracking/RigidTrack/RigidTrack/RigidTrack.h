#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_RigidTrack.h"
#include <qpixmap.h>
#include "main.h"

class RigidTrack : public QMainWindow
{
    Q_OBJECT

public:
    RigidTrack(QWidget *parent = Q_NULLPTR);

public slots:
	
	void on_btnStartCamera_clicked();
	void on_btnStopCamera_clicked();
	void on_btnZero_clicked();
	void on_btnCalibrate_clicked();
	void setImage(QPixmap image);
	void on_btnLoadCalib_clicked();
	void setLog(QString logText);
	void on_sbHeadingOffset_valueChanged(double d);
	void on_leIPDrone_returnPressed();

private:
	Ui::RigidTrackClass ui;
    
};
