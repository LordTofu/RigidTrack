#ifndef EMITTING_H
#define EMITTING_H

#include <QObject>
#include <qpixmap.h>

class commObject : public QObject {
	Q_OBJECT

public: 
	void changeStatus(QString newText);
	void changeImage(QPixmap image);
	void addLog(QString LogText);
	void clearLog();
	void enableP3P(bool value);
	void progressUpdate(int value);
	

	signals:
	   void statusChanged(QString newText);
	   void imageChanged(QPixmap image);
	   void logAdded(QString LogText);
	   void logCleared();
	   void P3Penabled(bool value);
	   void progressUpdated(int value);

};
#endif // EMITTING_H