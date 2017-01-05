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
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
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
    QLabel *lbLog;
    QPushButton *btnLoadCalib;
    QListWidget *listLog;
    QGroupBox *groupBox;
    QRadioButton *rbMethod4;
    QRadioButton *rbMethod1;
    QRadioButton *rbMethod2;
    QRadioButton *rbMethod3;
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
        btnStartCamera->setGeometry(QRect(20, 120, 221, 41));
        lbStatus = new QLabel(centralWidget);
        lbStatus->setObjectName(QStringLiteral("lbStatus"));
        lbStatus->setEnabled(true);
        lbStatus->setGeometry(QRect(30, 190, 771, 461));
        btnZero = new QPushButton(centralWidget);
        btnZero->setObjectName(QStringLiteral("btnZero"));
        btnZero->setGeometry(QRect(20, 70, 221, 41));
        btnCalibrate = new QPushButton(centralWidget);
        btnCalibrate->setObjectName(QStringLiteral("btnCalibrate"));
        btnCalibrate->setGeometry(QRect(20, 20, 101, 41));
        lbLog = new QLabel(centralWidget);
        lbLog->setObjectName(QStringLiteral("lbLog"));
        lbLog->setGeometry(QRect(270, 10, 231, 41));
        btnLoadCalib = new QPushButton(centralWidget);
        btnLoadCalib->setObjectName(QStringLiteral("btnLoadCalib"));
        btnLoadCalib->setGeometry(QRect(140, 20, 101, 41));
        listLog = new QListWidget(centralWidget);
        listLog->setObjectName(QStringLiteral("listLog"));
        listLog->setGeometry(QRect(20, 681, 791, 171));
        listLog->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setEnabled(false);
        groupBox->setGeometry(QRect(260, 50, 101, 111));
        rbMethod4 = new QRadioButton(groupBox);
        rbMethod4->setObjectName(QStringLiteral("rbMethod4"));
        rbMethod4->setGeometry(QRect(20, 80, 82, 17));
        rbMethod1 = new QRadioButton(groupBox);
        rbMethod1->setObjectName(QStringLiteral("rbMethod1"));
        rbMethod1->setGeometry(QRect(20, 20, 82, 17));
        rbMethod1->setChecked(true);
        rbMethod2 = new QRadioButton(groupBox);
        rbMethod2->setObjectName(QStringLiteral("rbMethod2"));
        rbMethod2->setGeometry(QRect(20, 40, 82, 17));
        rbMethod3 = new QRadioButton(groupBox);
        rbMethod3->setObjectName(QStringLiteral("rbMethod3"));
        rbMethod3->setGeometry(QRect(20, 60, 82, 17));
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
        lbLog->setText(QApplication::translate("RigidTrackClass", "TextLabel", 0));
        btnLoadCalib->setText(QApplication::translate("RigidTrackClass", "Load\n"
"Calibration", 0));
        groupBox->setTitle(QApplication::translate("RigidTrackClass", "PNP Method", 0));
        rbMethod4->setText(QApplication::translate("RigidTrackClass", "UPNP", 0));
        rbMethod1->setText(QApplication::translate("RigidTrackClass", "Iterative", 0));
        rbMethod2->setText(QApplication::translate("RigidTrackClass", "EPNP", 0));
        rbMethod3->setText(QApplication::translate("RigidTrackClass", "P3P", 0));
    } // retranslateUi

};

namespace Ui {
    class RigidTrackClass: public Ui_RigidTrackClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RIGIDTRACK_H
