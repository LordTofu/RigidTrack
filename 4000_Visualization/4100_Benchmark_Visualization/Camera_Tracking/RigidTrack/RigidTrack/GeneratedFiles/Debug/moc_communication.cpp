/****************************************************************************
** Meta object code from reading C++ file 'communication.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../communication.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'communication.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_commObject_t {
    QByteArrayData data[12];
    char stringdata0[114];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_commObject_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_commObject_t qt_meta_stringdata_commObject = {
    {
QT_MOC_LITERAL(0, 0, 10), // "commObject"
QT_MOC_LITERAL(1, 11, 13), // "statusChanged"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 7), // "newText"
QT_MOC_LITERAL(4, 34, 12), // "imageChanged"
QT_MOC_LITERAL(5, 47, 5), // "image"
QT_MOC_LITERAL(6, 53, 8), // "logAdded"
QT_MOC_LITERAL(7, 62, 7), // "LogText"
QT_MOC_LITERAL(8, 70, 10), // "logCleared"
QT_MOC_LITERAL(9, 81, 10), // "P3Penabled"
QT_MOC_LITERAL(10, 92, 5), // "value"
QT_MOC_LITERAL(11, 98, 15) // "progressUpdated"

    },
    "commObject\0statusChanged\0\0newText\0"
    "imageChanged\0image\0logAdded\0LogText\0"
    "logCleared\0P3Penabled\0value\0progressUpdated"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_commObject[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,
       4,    1,   47,    2, 0x06 /* Public */,
       6,    1,   50,    2, 0x06 /* Public */,
       8,    0,   53,    2, 0x06 /* Public */,
       9,    1,   54,    2, 0x06 /* Public */,
      11,    1,   57,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QPixmap,    5,
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void, QMetaType::Int,   10,

       0        // eod
};

void commObject::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        commObject *_t = static_cast<commObject *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->statusChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->imageChanged((*reinterpret_cast< QPixmap(*)>(_a[1]))); break;
        case 2: _t->logAdded((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->logCleared(); break;
        case 4: _t->P3Penabled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->progressUpdated((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (commObject::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&commObject::statusChanged)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (commObject::*_t)(QPixmap );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&commObject::imageChanged)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (commObject::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&commObject::logAdded)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (commObject::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&commObject::logCleared)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (commObject::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&commObject::P3Penabled)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (commObject::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&commObject::progressUpdated)) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject commObject::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_commObject.data,
      qt_meta_data_commObject,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *commObject::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *commObject::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_commObject.stringdata0))
        return static_cast<void*>(const_cast< commObject*>(this));
    return QObject::qt_metacast(_clname);
}

int commObject::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void commObject::statusChanged(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void commObject::imageChanged(QPixmap _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void commObject::logAdded(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void commObject::logCleared()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void commObject::P3Penabled(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void commObject::progressUpdated(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
