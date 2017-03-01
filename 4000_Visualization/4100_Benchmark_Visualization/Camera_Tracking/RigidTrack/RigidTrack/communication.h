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
	

	signals:
	   void statusChanged(QString newText);
	   void imageChanged(QPixmap image);
	   void logAdded(QString LogText);

};
#endif // EMITTING_H