/********************************************************************************
** Form generated from reading UI file 'RigidTrack.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RIGIDTRACK_H
#define UI_RIGIDTRACK_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RigidTrackClass
{
public:
    QWidget *centralWidget;
    QPushButton *btnStartCamera;
    QLabel *lbStatus;
    QPushButton *btnZero;
    QPushButton *btnCalibrate;
    QPushButton *btnLoadCalib;
    QListWidget *listLog;
    QDoubleSpinBox *sbHeadingOffset;
    QLabel *label;
    QLineEdit *leIPDrone;
    QLabel *label_2;
    QPushButton *btnStopCamera;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *RigidTrackClass)
    {
        if (RigidTrackClass->objectName().isEmpty())
            RigidTrackClass->setObjectName(QStringLiteral("RigidTrackClass"));
        RigidTrackClass->resize(830, 908);
        centralWidget = new QWidget(RigidTrackClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        btnStartCamera = new QPushButton(centralWidget);
        btnStartCamera->setObjectName(QStringLiteral("btnStartCamera"));
        btnStartCamera->setGeometry(QRect(20, 80, 221, 41));
        lbStatus = new QLabel(centralWidget);
        lbStatus->setObjectName(QStringLiteral("lbStatus"));
        lbStatus->setEnabled(true);
        lbStatus->setGeometry(QRect(30, 190, 771, 461));
        btnZero = new QPushButton(centralWidget);
        btnZero->setObjectName(QStringLiteral("btnZero"));
        btnZero->setGeometry(QRect(20, 30, 221, 41));
        btnCalibrate = new QPushButton(centralWidget);
        btnCalibrate->setObjectName(QStringLiteral("btnCalibrate"));
        btnCalibrate->setGeometry(QRect(420, 30, 101, 41));
        btnLoadCalib = new QPushButton(centralWidget);
        btnLoadCalib->setObjectName(QStringLiteral("btnLoadCalib"));
        btnLoadCalib->setGeometry(QRect(530, 30, 101, 41));
        listLog = new QListWidget(centralWidget);
        listLog->setObjectName(QStringLiteral("listLog"));
        listLog->setGeometry(QRect(20, 681, 791, 171));
        listLog->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        sbHeadingOffset = new QDoubleSpinBox(centralWidget);
        sbHeadingOffset->setObjectName(QStringLiteral("sbHeadingOffset"));
        sbHeadingOffset->setGeometry(QRect(260, 40, 136, 20));
        sbHeadingOffset->setDecimals(1);
        sbHeadingOffset->setMinimum(-180);
        sbHeadingOffset->setMaximum(180);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(260, 20, 136, 21));
        leIPDrone = new QLineEdit(centralWidget);
        leIPDrone->setObjectName(QStringLiteral("leIPDrone"));
        leIPDrone->setGeometry(QRect(260, 100, 136, 20));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(260, 70, 141, 26));
        label_2->setWordWrap(true);
        btnStopCamera = new QPushButton(centralWidget);
        btnStopCamera->setObjectName(QStringLiteral("btnStopCamera"));
        btnStopCamera->setGeometry(QRect(20, 130, 221, 41));
        btnStopCamera->setFocusPolicy(Qt::ClickFocus);
        RigidTrackClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(RigidTrackClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 830, 21));
        RigidTrackClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(RigidTrackClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        RigidTrackClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(RigidTrackClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        RigidTrackClass->setStatusBar(statusBar);

        retranslateUi(RigidTrackClass);

        QMetaObject::connectSlotsByName(RigidTrackClass);
    } // setupUi

    void retranslateUi(QMainWindow *RigidTrackClass)
    {
        RigidTrackClass->setWindowTitle(QApplication::translate("RigidTrackClass", "RigidTrack", 0));
        btnStartCamera->setText(QApplication::translate("RigidTrackClass", "Start Tracking", 0));
        lbStatus->setText(QApplication::translate("RigidTrackClass", "TextLabel", 0));
        btnZero->setText(QApplication::translate("RigidTrackClass", "Set Reference Point", 0));
        btnCalibrate->setText(QApplication::translate("RigidTrackClass", "Calibrate\n"
"Camera", 0));
        btnLoadCalib->setText(QApplication::translate("RigidTrackClass", "Load\n"
"Calibration", 0));
        label->setText(QApplication::translate("RigidTrackClass", "Heading Offset", 0));
        leIPDrone->setText(QApplication::translate("RigidTrackClass", "192.168.4.2", 0));
        label_2->setText(QApplication::translate("RigidTrackClass", "IP Adress of Drone Wifi Chip Press Return to Apply Changes", 0));
        btnStopCamera->setText(QApplication::translate("RigidTrackClass", "Stop Tracking", 0));
    } // retranslateUi

};

namespace Ui {
    class RigidTrackClass: public Ui_RigidTrackClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RIGIDTRACK_H
