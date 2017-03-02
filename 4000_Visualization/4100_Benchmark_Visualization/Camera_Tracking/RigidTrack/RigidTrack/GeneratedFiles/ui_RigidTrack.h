/********************************************************************************
** Form generated from reading UI file 'RigidTrack.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
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
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RigidTrackClass
{
public:
    QAction *actionShow_Help;
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
    QLabel *lbSafetyArea;
    QSpinBox *spinBox_2;
    QDoubleSpinBox *doubleSpinBox;
    QLabel *lbSafetyArea_2;
    QMenuBar *menuBar;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *RigidTrackClass)
    {
        if (RigidTrackClass->objectName().isEmpty())
            RigidTrackClass->setObjectName(QStringLiteral("RigidTrackClass"));
        RigidTrackClass->resize(830, 908);
        actionShow_Help = new QAction(RigidTrackClass);
        actionShow_Help->setObjectName(QStringLiteral("actionShow_Help"));
        centralWidget = new QWidget(RigidTrackClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        btnStartCamera = new QPushButton(centralWidget);
        btnStartCamera->setObjectName(QStringLiteral("btnStartCamera"));
        btnStartCamera->setGeometry(QRect(20, 80, 131, 41));
        lbStatus = new QLabel(centralWidget);
        lbStatus->setObjectName(QStringLiteral("lbStatus"));
        lbStatus->setEnabled(true);
        lbStatus->setGeometry(QRect(30, 190, 771, 461));
        btnZero = new QPushButton(centralWidget);
        btnZero->setObjectName(QStringLiteral("btnZero"));
        btnZero->setGeometry(QRect(20, 30, 131, 41));
        btnCalibrate = new QPushButton(centralWidget);
        btnCalibrate->setObjectName(QStringLiteral("btnCalibrate"));
        btnCalibrate->setGeometry(QRect(330, 30, 101, 41));
        btnLoadCalib = new QPushButton(centralWidget);
        btnLoadCalib->setObjectName(QStringLiteral("btnLoadCalib"));
        btnLoadCalib->setGeometry(QRect(330, 80, 101, 41));
        listLog = new QListWidget(centralWidget);
        listLog->setObjectName(QStringLiteral("listLog"));
        listLog->setGeometry(QRect(20, 681, 791, 171));
        listLog->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        sbHeadingOffset = new QDoubleSpinBox(centralWidget);
        sbHeadingOffset->setObjectName(QStringLiteral("sbHeadingOffset"));
        sbHeadingOffset->setGeometry(QRect(170, 40, 136, 20));
        sbHeadingOffset->setDecimals(1);
        sbHeadingOffset->setMinimum(-180);
        sbHeadingOffset->setMaximum(180);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(170, 20, 136, 21));
        leIPDrone = new QLineEdit(centralWidget);
        leIPDrone->setObjectName(QStringLiteral("leIPDrone"));
        leIPDrone->setGeometry(QRect(170, 100, 136, 20));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(170, 70, 141, 26));
        label_2->setWordWrap(true);
        lbSafetyArea = new QLabel(centralWidget);
        lbSafetyArea->setObjectName(QStringLiteral("lbSafetyArea"));
        lbSafetyArea->setGeometry(QRect(460, 10, 111, 31));
        lbSafetyArea->setWordWrap(true);
        spinBox_2 = new QSpinBox(centralWidget);
        spinBox_2->setObjectName(QStringLiteral("spinBox_2"));
        spinBox_2->setGeometry(QRect(460, 90, 42, 22));
        doubleSpinBox = new QDoubleSpinBox(centralWidget);
        doubleSpinBox->setObjectName(QStringLiteral("doubleSpinBox"));
        doubleSpinBox->setGeometry(QRect(460, 40, 62, 22));
        lbSafetyArea_2 = new QLabel(centralWidget);
        lbSafetyArea_2->setObjectName(QStringLiteral("lbSafetyArea_2"));
        lbSafetyArea_2->setGeometry(QRect(460, 60, 111, 31));
        lbSafetyArea_2->setWordWrap(true);
        RigidTrackClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(RigidTrackClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 830, 21));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        RigidTrackClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(RigidTrackClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        RigidTrackClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(RigidTrackClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        RigidTrackClass->setStatusBar(statusBar);

        menuBar->addAction(menuHelp->menuAction());
        menuHelp->addAction(actionShow_Help);

        retranslateUi(RigidTrackClass);

        QMetaObject::connectSlotsByName(RigidTrackClass);
    } // setupUi

    void retranslateUi(QMainWindow *RigidTrackClass)
    {
        RigidTrackClass->setWindowTitle(QApplication::translate("RigidTrackClass", "RigidTrack", Q_NULLPTR));
        actionShow_Help->setText(QApplication::translate("RigidTrackClass", "Show Help", Q_NULLPTR));
        btnStartCamera->setText(QApplication::translate("RigidTrackClass", "Start Tracking", Q_NULLPTR));
        lbStatus->setText(QApplication::translate("RigidTrackClass", "TextLabel", Q_NULLPTR));
        btnZero->setText(QApplication::translate("RigidTrackClass", "Set Reference Point", Q_NULLPTR));
        btnCalibrate->setText(QApplication::translate("RigidTrackClass", "Calibrate\n"
"Camera", Q_NULLPTR));
        btnLoadCalib->setText(QApplication::translate("RigidTrackClass", "Load\n"
"Calibration", Q_NULLPTR));
        label->setText(QApplication::translate("RigidTrackClass", "Heading Offset", Q_NULLPTR));
        leIPDrone->setText(QApplication::translate("RigidTrackClass", "192.168.4.2", Q_NULLPTR));
        label_2->setText(QApplication::translate("RigidTrackClass", "IP Adress of Drone Wifi Chip Press Return to Apply Changes", Q_NULLPTR));
        lbSafetyArea->setText(QApplication::translate("RigidTrackClass", "Safety Area Dimensions ", Q_NULLPTR));
        lbSafetyArea_2->setText(QApplication::translate("RigidTrackClass", "Safety Area Angles", Q_NULLPTR));
        menuHelp->setTitle(QApplication::translate("RigidTrackClass", "Help", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class RigidTrackClass: public Ui_RigidTrackClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RIGIDTRACK_H
