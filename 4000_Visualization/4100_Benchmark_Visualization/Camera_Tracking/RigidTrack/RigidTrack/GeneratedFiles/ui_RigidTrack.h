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
#include <QtWidgets/QProgressBar>
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
    QAction *actionOpen_Log_Folder;
    QAction *actionOpen_Installation_Folder;
    QWidget *centralWidget;
    QPushButton *btnStartCamera;
    QLabel *lbStatus;
    QPushButton *btnZero;
    QPushButton *btnCalibrate;
    QPushButton *btnLoadCalib;
    QListWidget *listLog;
    QDoubleSpinBox *sbHeadingOffset;
    QLabel *label;
    QLineEdit *leIPObject;
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
    QLineEdit *leIPSafety;
    QLabel *label_3;
    QPushButton *pbLoadMarker;
    QGroupBox *groupBox_3;
    QCheckBox *cbSafety2;
    QLineEdit *leIPSafety2;
    QCheckBox *cbInvert;
    QProgressBar *progressBar;
    QPushButton *btnCalibrateGround;
    QMenuBar *menuBar;
    QMenu *menuHelp;
    QMenu *menuOpen_Logs;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *RigidTrackClass)
    {
        if (RigidTrackClass->objectName().isEmpty())
            RigidTrackClass->setObjectName(QStringLiteral("RigidTrackClass"));
        RigidTrackClass->resize(876, 924);
        RigidTrackClass->setWindowTitle(QStringLiteral("Rigid Track"));
        QIcon icon;
        icon.addFile(QStringLiteral("icon.ico"), QSize(), QIcon::Normal, QIcon::Off);
        RigidTrackClass->setWindowIcon(icon);
        actionShow_Help = new QAction(RigidTrackClass);
        actionShow_Help->setObjectName(QStringLiteral("actionShow_Help"));
        actionOpen_Log_Folder = new QAction(RigidTrackClass);
        actionOpen_Log_Folder->setObjectName(QStringLiteral("actionOpen_Log_Folder"));
        actionOpen_Installation_Folder = new QAction(RigidTrackClass);
        actionOpen_Installation_Folder->setObjectName(QStringLiteral("actionOpen_Installation_Folder"));
        centralWidget = new QWidget(RigidTrackClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        btnStartCamera = new QPushButton(centralWidget);
        btnStartCamera->setObjectName(QStringLiteral("btnStartCamera"));
        btnStartCamera->setGeometry(QRect(20, 60, 111, 41));
        lbStatus = new QLabel(centralWidget);
        lbStatus->setObjectName(QStringLiteral("lbStatus"));
        lbStatus->setEnabled(true);
        lbStatus->setGeometry(QRect(120, 180, 640, 480));
        btnZero = new QPushButton(centralWidget);
        btnZero->setObjectName(QStringLiteral("btnZero"));
        btnZero->setGeometry(QRect(20, 10, 111, 41));
        btnCalibrate = new QPushButton(centralWidget);
        btnCalibrate->setObjectName(QStringLiteral("btnCalibrate"));
        btnCalibrate->setGeometry(QRect(160, 10, 101, 41));
        btnLoadCalib = new QPushButton(centralWidget);
        btnLoadCalib->setObjectName(QStringLiteral("btnLoadCalib"));
        btnLoadCalib->setGeometry(QRect(160, 60, 101, 41));
        listLog = new QListWidget(centralWidget);
        listLog->setObjectName(QStringLiteral("listLog"));
        listLog->setGeometry(QRect(10, 700, 851, 170));
        listLog->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        sbHeadingOffset = new QDoubleSpinBox(centralWidget);
        sbHeadingOffset->setObjectName(QStringLiteral("sbHeadingOffset"));
        sbHeadingOffset->setGeometry(QRect(280, 80, 71, 20));
        sbHeadingOffset->setDecimals(1);
        sbHeadingOffset->setMinimum(-180);
        sbHeadingOffset->setMaximum(180);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(280, 60, 81, 21));
        leIPObject = new QLineEdit(centralWidget);
        leIPObject->setObjectName(QStringLiteral("leIPObject"));
        leIPObject->setGeometry(QRect(390, 50, 136, 20));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(390, 20, 141, 26));
        label_2->setWordWrap(true);
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(770, 30, 101, 71));
        rbIterative = new QRadioButton(groupBox);
        rbIterative->setObjectName(QStringLiteral("rbIterative"));
        rbIterative->setGeometry(QRect(10, 40, 82, 17));
        rbIterative->setChecked(true);
        rbP3P = new QRadioButton(groupBox);
        rbP3P->setObjectName(QStringLiteral("rbP3P"));
        rbP3P->setGeometry(QRect(10, 20, 82, 17));
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(580, 10, 171, 161));
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
        leIPSafety = new QLineEdit(groupBox_2);
        leIPSafety->setObjectName(QStringLiteral("leIPSafety"));
        leIPSafety->setEnabled(false);
        leIPSafety->setGeometry(QRect(10, 130, 136, 20));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 100, 161, 26));
        label_3->setWordWrap(true);
        pbLoadMarker = new QPushButton(centralWidget);
        pbLoadMarker->setObjectName(QStringLiteral("pbLoadMarker"));
        pbLoadMarker->setGeometry(QRect(160, 110, 101, 41));
        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(380, 80, 181, 80));
        cbSafety2 = new QCheckBox(groupBox_3);
        cbSafety2->setObjectName(QStringLiteral("cbSafety2"));
        cbSafety2->setGeometry(QRect(10, 20, 151, 17));
        leIPSafety2 = new QLineEdit(groupBox_3);
        leIPSafety2->setObjectName(QStringLiteral("leIPSafety2"));
        leIPSafety2->setEnabled(false);
        leIPSafety2->setGeometry(QRect(10, 40, 136, 20));
        cbInvert = new QCheckBox(centralWidget);
        cbInvert->setObjectName(QStringLiteral("cbInvert"));
        cbInvert->setGeometry(QRect(280, 110, 70, 17));
        progressBar = new QProgressBar(centralWidget);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        progressBar->setGeometry(QRect(10, 670, 850, 20));
        progressBar->setValue(0);
        progressBar->setTextVisible(true);
        progressBar->setOrientation(Qt::Horizontal);
        btnCalibrateGround = new QPushButton(centralWidget);
        btnCalibrateGround->setObjectName(QStringLiteral("btnCalibrateGround"));
        btnCalibrateGround->setGeometry(QRect(270, 10, 101, 41));
        RigidTrackClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(RigidTrackClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 876, 21));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        menuOpen_Logs = new QMenu(menuBar);
        menuOpen_Logs->setObjectName(QStringLiteral("menuOpen_Logs"));
        RigidTrackClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(RigidTrackClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        RigidTrackClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(RigidTrackClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        RigidTrackClass->setStatusBar(statusBar);

        menuBar->addAction(menuHelp->menuAction());
        menuBar->addAction(menuOpen_Logs->menuAction());
        menuHelp->addAction(actionShow_Help);
        menuOpen_Logs->addAction(actionOpen_Log_Folder);
        menuOpen_Logs->addAction(actionOpen_Installation_Folder);

        retranslateUi(RigidTrackClass);

        QMetaObject::connectSlotsByName(RigidTrackClass);
    } // setupUi

    void retranslateUi(QMainWindow *RigidTrackClass)
    {
        actionShow_Help->setText(QApplication::translate("RigidTrackClass", "Show Help", 0));
        actionOpen_Log_Folder->setText(QApplication::translate("RigidTrackClass", "Open Log Folder", 0));
        actionOpen_Installation_Folder->setText(QApplication::translate("RigidTrackClass", "Open Installation Folder", 0));
        btnStartCamera->setText(QApplication::translate("RigidTrackClass", "Start Tracking", 0));
        lbStatus->setText(QApplication::translate("RigidTrackClass", "TextLabel", 0));
        btnZero->setText(QApplication::translate("RigidTrackClass", "Set Reference Point", 0));
        btnCalibrate->setText(QApplication::translate("RigidTrackClass", "Calibrate\n"
"Camera", 0));
        btnLoadCalib->setText(QApplication::translate("RigidTrackClass", "Load\n"
"Calibration", 0));
        label->setText(QApplication::translate("RigidTrackClass", "Heading Offset", 0));
        leIPObject->setText(QApplication::translate("RigidTrackClass", "192.168.137.254:9155", 0));
        label_2->setText(QApplication::translate("RigidTrackClass", "IP Adress:Port of Object Wifi Chip Press Return to Apply Changes", 0));
        groupBox->setTitle(QApplication::translate("RigidTrackClass", "PnP Algorithm", 0));
        rbIterative->setText(QApplication::translate("RigidTrackClass", "Iterative", 0));
        rbP3P->setText(QApplication::translate("RigidTrackClass", "P3P", 0));
        groupBox_2->setTitle(QApplication::translate("RigidTrackClass", "Safety Protection", 0));
#ifndef QT_NO_TOOLTIP
        cbSafety->setToolTip(QApplication::translate("RigidTrackClass", "If this check box is enabled the software will monitor the position and attitude. If the values of safety area dimensions and safety area angles are exceeded a disable signal is sent via UDP.", 0));
#endif // QT_NO_TOOLTIP
        cbSafety->setText(QApplication::translate("RigidTrackClass", "Enable Safety Area", 0));
#ifndef QT_NO_TOOLTIP
        dsbDimension->setToolTip(QApplication::translate("RigidTrackClass", "Maximum distance of the object to the starting point in each dimension. If exceeded a disable signal is sent via UDP", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        sbAngle->setToolTip(QApplication::translate("RigidTrackClass", "Maximum bank and pitch angle of the object. If exceeded a disable signal is sent via UDP", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        lbSafetyArea->setToolTip(QApplication::translate("RigidTrackClass", "Maximum distance of the object to the starting point in each dimension. If exceeded a disable signal is sent via UDP", 0));
#endif // QT_NO_TOOLTIP
        lbSafetyArea->setText(QApplication::translate("RigidTrackClass", "Safety Area Dimensions ", 0));
#ifndef QT_NO_TOOLTIP
        lbSafetyArea_2->setToolTip(QApplication::translate("RigidTrackClass", "Maximum bank and pitch angle of the object. If exceeded a disable signal is sent via UDP", 0));
#endif // QT_NO_TOOLTIP
        lbSafetyArea_2->setText(QApplication::translate("RigidTrackClass", "Safety Area Angles", 0));
        leIPSafety->setText(QApplication::translate("RigidTrackClass", "192.168.137.2:9155", 0));
        label_3->setText(QApplication::translate("RigidTrackClass", "IP Adress of Emergency Switch Press Return to Apply", 0));
        pbLoadMarker->setText(QApplication::translate("RigidTrackClass", "Load Marker\n"
"Configuration", 0));
        groupBox_3->setTitle(QApplication::translate("RigidTrackClass", "Additional Receiver", 0));
#ifndef QT_NO_TOOLTIP
        cbSafety2->setToolTip(QApplication::translate("RigidTrackClass", "If this check box is enabled the software will monitor the position and attitude. If the values of safety area dimensions and safety area angles are exceeded a disable signal is sent via UDP.", 0));
#endif // QT_NO_TOOLTIP
        cbSafety2->setText(QApplication::translate("RigidTrackClass", "Send to second Receiver", 0));
        leIPSafety2->setText(QApplication::translate("RigidTrackClass", "192.168.137.254:9155", 0));
        cbInvert->setText(QApplication::translate("RigidTrackClass", "Invert Z", 0));
        btnCalibrateGround->setText(QApplication::translate("RigidTrackClass", "Calibrate\n"
"Ground", 0));
        menuHelp->setTitle(QApplication::translate("RigidTrackClass", "Help", 0));
        menuOpen_Logs->setTitle(QApplication::translate("RigidTrackClass", "Open Folders", 0));
        Q_UNUSED(RigidTrackClass);
    } // retranslateUi

};

namespace Ui {
    class RigidTrackClass: public Ui_RigidTrackClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RIGIDTRACK_H
