/****************************************************************************
** Meta object code from reading C++ file 'RigidTrack.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../RigidTrack.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RigidTrack.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RigidTrack_t {
    QByteArrayData data[16];
    char stringdata0[261];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RigidTrack_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RigidTrack_t qt_meta_stringdata_RigidTrack = {
    {
QT_MOC_LITERAL(0, 0, 10), // "RigidTrack"
QT_MOC_LITERAL(1, 11, 25), // "on_btnStartCamera_clicked"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 24), // "on_btnStopCamera_clicked"
QT_MOC_LITERAL(4, 63, 18), // "on_btnZero_clicked"
QT_MOC_LITERAL(5, 82, 23), // "on_btnCalibrate_clicked"
QT_MOC_LITERAL(6, 106, 8), // "setImage"
QT_MOC_LITERAL(7, 115, 5), // "image"
QT_MOC_LITERAL(8, 121, 23), // "on_btnLoadCalib_clicked"
QT_MOC_LITERAL(9, 145, 6), // "setLog"
QT_MOC_LITERAL(10, 152, 7), // "logText"
QT_MOC_LITERAL(11, 160, 31), // "on_sbHeadingOffset_valueChanged"
QT_MOC_LITERAL(12, 192, 1), // "d"
QT_MOC_LITERAL(13, 194, 26), // "on_leIPDrone_returnPressed"
QT_MOC_LITERAL(14, 221, 16), // "on_rbP3P_clicked"
QT_MOC_LITERAL(15, 238, 22) // "on_rbIterative_clicked"

    },
    "RigidTrack\0on_btnStartCamera_clicked\0"
    "\0on_btnStopCamera_clicked\0on_btnZero_clicked\0"
    "on_btnCalibrate_clicked\0setImage\0image\0"
    "on_btnLoadCalib_clicked\0setLog\0logText\0"
    "on_sbHeadingOffset_valueChanged\0d\0"
    "on_leIPDrone_returnPressed\0on_rbP3P_clicked\0"
    "on_rbIterative_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RigidTrack[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x0a /* Public */,
       3,    0,   70,    2, 0x0a /* Public */,
       4,    0,   71,    2, 0x0a /* Public */,
       5,    0,   72,    2, 0x0a /* Public */,
       6,    1,   73,    2, 0x0a /* Public */,
       8,    0,   76,    2, 0x0a /* Public */,
       9,    1,   77,    2, 0x0a /* Public */,
      11,    1,   80,    2, 0x0a /* Public */,
      13,    0,   83,    2, 0x0a /* Public */,
      14,    0,   84,    2, 0x0a /* Public */,
      15,    0,   85,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QPixmap,    7,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void, QMetaType::Double,   12,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RigidTrack::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RigidTrack *_t = static_cast<RigidTrack *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_btnStartCamera_clicked(); break;
        case 1: _t->on_btnStopCamera_clicked(); break;
        case 2: _t->on_btnZero_clicked(); break;
        case 3: _t->on_btnCalibrate_clicked(); break;
        case 4: _t->setImage((*reinterpret_cast< QPixmap(*)>(_a[1]))); break;
        case 5: _t->on_btnLoadCalib_clicked(); break;
        case 6: _t->setLog((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->on_sbHeadingOffset_valueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: _t->on_leIPDrone_returnPressed(); break;
        case 9: _t->on_rbP3P_clicked(); break;
        case 10: _t->on_rbIterative_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject RigidTrack::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_RigidTrack.data,
      qt_meta_data_RigidTrack,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RigidTrack::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RigidTrack::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RigidTrack.stringdata0))
        return static_cast<void*>(const_cast< RigidTrack*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int RigidTrack::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
