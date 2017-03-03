#include <QObject>
#include <QString>
#include <QtWidgets/QApplication>
#include <qpixmap.h>
#include <stdio.h>
#include "communication.h"

void commObject::changeStatus(QString newText) {

	emit statusChanged(newText);
	QCoreApplication::processEvents();
}

void commObject::changeImage(QPixmap image) {

	emit imageChanged(image);
	QCoreApplication::processEvents();
}

void commObject::addLog(QString LogText) {

	emit logAdded(LogText);
	QCoreApplication::processEvents();
}

void commObject::clearLog() {

	emit logCleared();
	QCoreApplication::processEvents();
}