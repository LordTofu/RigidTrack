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
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
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
    QGroupBox *groupBox;
    QRadioButton *rbIterative;
    QRadioButton *rbP3P;
    QGroupBox *groupBox_2;
    QCheckBox *cbSafety;
    QDoubleSpinBox *dsbDimension;
    QSpinBox *sbAngle;
    QLabel *lbSafetyArea;
    QLabel *lbSafetyArea_2;
    QLineEdit *leIPCB;
    QLabel *label_3;
    QMenuBar *menuBar;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *RigidTrackClass)
    {
        if (RigidTrackClass->objectName().isEmpty())
            RigidTrackClass->setObjectName(QStringLiteral("RigidTrackClass"));
        RigidTrackClass->resize(740, 900);
        RigidTrackClass->setWindowTitle(QStringLiteral("Optical Position Tracking"));
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
        lbStatus->setGeometry(QRect(50, 180, 640, 480));
        btnZero = new QPushButton(centralWidget);
        btnZero->setObjectName(QStringLiteral("btnZero"));
        btnZero->setGeometry(QRect(20, 30, 131, 41));
        btnCalibrate = new QPushButton(centralWidget);
        btnCalibrate->setObjectName(QStringLiteral("btnCalibrate"));
        btnCalibrate->setGeometry(QRect(160, 30, 101, 41));
        btnLoadCalib = new QPushButton(centralWidget);
        btnLoadCalib->setObjectName(QStringLiteral("btnLoadCalib"));
        btnLoadCalib->setGeometry(QRect(160, 80, 101, 41));
        listLog = new QListWidget(centralWidget);
        listLog->setObjectName(QStringLiteral("listLog"));
        listLog->setGeometry(QRect(10, 680, 720, 170));
        listLog->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        sbHeadingOffset = new QDoubleSpinBox(centralWidget);
        sbHeadingOffset->setObjectName(QStringLiteral("sbHeadingOffset"));
        sbHeadingOffset->setGeometry(QRect(280, 50, 71, 20));
        sbHeadingOffset->setDecimals(1);
        sbHeadingOffset->setMinimum(-180);
        sbHeadingOffset->setMaximum(180);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(280, 30, 136, 21));
        leIPDrone = new QLineEdit(centralWidget);
        leIPDrone->setObjectName(QStringLiteral("leIPDrone"));
        leIPDrone->setGeometry(QRect(280, 110, 136, 20));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(280, 80, 141, 26));
        label_2->setWordWrap(true);
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(630, 30, 101, 71));
        rbIterative = new QRadioButton(groupBox);
        rbIterative->setObjectName(QStringLiteral("rbIterative"));
        rbIterative->setGeometry(QRect(10, 40, 82, 17));
        rbIterative->setChecked(true);
        rbP3P = new QRadioButton(groupBox);
        rbP3P->setObjectName(QStringLiteral("rbP3P"));
        rbP3P->setGeometry(QRect(10, 20, 82, 17));
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(440, 10, 171, 161));
        cbSafety = new QCheckBox(groupBox_2);
        cbSafety->setObjectName(QStringLiteral("cbSafety"));
        cbSafety->setGeometry(QRect(10, 20, 131, 17));
        dsbDimension = new QDoubleSpinBox(groupBox_2);
        dsbDimension->setObjectName(QStringLiteral("dsbDimension"));
        dsbDimension->setEnabled(false);
        dsbDimension->setGeometry(QRect(10, 70, 62, 22));
        sbAngle = new QSpinBox(groupBox_2);
        sbAngle->setObjectName(QStringLiteral("sbAngle"));
        sbAngle->setEnabled(false);
        sbAngle->setGeometry(QRect(90, 70, 61, 22));
        lbSafetyArea = new QLabel(groupBox_2);
        lbSafetyArea->setObjectName(QStringLiteral("lbSafetyArea"));
        lbSafetyArea->setGeometry(QRect(10, 40, 61, 31));
        lbSafetyArea->setWordWrap(true);
        lbSafetyArea_2 = new QLabel(groupBox_2);
        lbSafetyArea_2->setObjectName(QStringLiteral("lbSafetyArea_2"));
        lbSafetyArea_2->setGeometry(QRect(90, 40, 91, 31));
        lbSafetyArea_2->setWordWrap(true);
        leIPCB = new QLineEdit(groupBox_2);
        leIPCB->setObjectName(QStringLiteral("leIPCB"));
        leIPCB->setEnabled(false);
        leIPCB->setGeometry(QRect(10, 130, 136, 20));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 100, 161, 26));
        label_3->setWordWrap(true);
        RigidTrackClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(RigidTrackClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 740, 21));
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
        actionShow_Help->setText(QApplication::translate("RigidTrackClass", "Show Help", Q_NULLPTR));
        btnStartCamera->setText(QApplication::translate("RigidTrackClass", "Start Tracking", Q_NULLPTR));
        lbStatus->setText(QApplication::translate("RigidTrackClass", "TextLabel", Q_NULLPTR));
        btnZero->setText(QApplication::translate("RigidTrackClass", "Set Reference Point", Q_NULLPTR));
        btnCalibrate->setText(QApplication::translate("RigidTrackClass", "Calibrate\n"
"Camera", Q_NULLPTR));
        btnLoadCalib->setText(QApplication::translate("RigidTrackClass", "Load\n"
"Calibration", Q_NULLPTR));
        label->setText(QApplication::translate("RigidTrackClass", "Heading Offset", Q_NULLPTR));
        leIPDrone->setText(QApplication::translate("RigidTrackClass", "192.168.137.254", Q_NULLPTR));
        label_2->setText(QApplication::translate("RigidTrackClass", "IP Adress of Drone Wifi Chip Press Return to Apply Changes", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("RigidTrackClass", "PnP Algorithm", Q_NULLPTR));
        rbIterative->setText(QApplication::translate("RigidTrackClass", "Iterative", Q_NULLPTR));
        rbP3P->setText(QApplication::translate("RigidTrackClass", "P3P", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("RigidTrackClass", "Safety Protection", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        cbSafety->setToolTip(QApplication::translate("RigidTrackClass", "If this check box is enabled the software will monitor the position and attitude. If the values of safety area dimensions and safety area angles are exceeded a disable signal is sent via UDP.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        cbSafety->setText(QApplication::translate("RigidTrackClass", "Enable Safety Area", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        dsbDimension->setToolTip(QApplication::translate("RigidTrackClass", "Maximum distance of the object to the starting point in each dimension. If exceeded a disable signal is sent via UDP", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        sbAngle->setToolTip(QApplication::translate("RigidTrackClass", "Maximum bank and pitch angle of the object. If exceeded a disable signal is sent via UDP", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        lbSafetyArea->setToolTip(QApplication::translate("RigidTrackClass", "Maximum distance of the object to the starting point in each dimension. If exceeded a disable signal is sent via UDP", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        lbSafetyArea->setText(QApplication::translate("RigidTrackClass", "Safety Area Dimensions ", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        lbSafetyArea_2->setToolTip(QApplication::translate("RigidTrackClass", "Maximum bank and pitch angle of the object. If exceeded a disable signal is sent via UDP", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        lbSafetyArea_2->setText(QApplication::translate("RigidTrackClass", "Safety Area Angles", Q_NULLPTR));
        leIPCB->setText(QApplication::translate("RigidTrackClass", "192.168.137.2", Q_NULLPTR));
        label_3->setText(QApplication::translate("RigidTrackClass", "IP Adress of Emergency Switch Press Return to Apply", Q_NULLPTR));
        menuHelp->setTitle(QApplication::translate("RigidTrackClass", "Help", Q_NULLPTR));
        Q_UNUSED(RigidTrackClass);
    } // retranslateUi

};

namespace Ui {
    class RigidTrackClass: public Ui_RigidTrackClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RIGIDTRACK_H
