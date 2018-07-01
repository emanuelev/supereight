/****************************************************************************
** Meta object code from reading C++ file 'ApplicationWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.2.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "qt/ApplicationWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ApplicationWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.2.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ButtonChoices_t {
    QByteArrayData data[5];
    char stringdata[52];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_ButtonChoices_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_ButtonChoices_t qt_meta_stringdata_ButtonChoices = {
    {
QT_MOC_LITERAL(0, 0, 13),
QT_MOC_LITERAL(1, 14, 19),
QT_MOC_LITERAL(2, 34, 0),
QT_MOC_LITERAL(3, 35, 8),
QT_MOC_LITERAL(4, 44, 6)
    },
    "ButtonChoices\0changeButtonChoices\0\0"
    "QAction*\0action\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ButtonChoices[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x08,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void ButtonChoices::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ButtonChoices *_t = static_cast<ButtonChoices *>(_o);
        switch (_id) {
        case 0: _t->changeButtonChoices((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QAction* >(); break;
            }
            break;
        }
    }
}

const QMetaObject ButtonChoices::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_ButtonChoices.data,
      qt_meta_data_ButtonChoices,  qt_static_metacall, 0, 0}
};


const QMetaObject *ButtonChoices::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ButtonChoices::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ButtonChoices.stringdata))
        return static_cast<void*>(const_cast< ButtonChoices*>(this));
    return QWidget::qt_metacast(_clname);
}

int ButtonChoices::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
struct qt_meta_stringdata_ApplicationWindow_t {
    QByteArrayData data[34];
    char stringdata[393];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_ApplicationWindow_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_ApplicationWindow_t qt_meta_stringdata_ApplicationWindow = {
    {
QT_MOC_LITERAL(0, 0, 17),
QT_MOC_LITERAL(1, 18, 13),
QT_MOC_LITERAL(2, 32, 0),
QT_MOC_LITERAL(3, 33, 10),
QT_MOC_LITERAL(4, 44, 5),
QT_MOC_LITERAL(5, 50, 10),
QT_MOC_LITERAL(6, 61, 12),
QT_MOC_LITERAL(7, 74, 12),
QT_MOC_LITERAL(8, 87, 10),
QT_MOC_LITERAL(9, 98, 12),
QT_MOC_LITERAL(10, 111, 12),
QT_MOC_LITERAL(11, 124, 12),
QT_MOC_LITERAL(12, 137, 12),
QT_MOC_LITERAL(13, 150, 12),
QT_MOC_LITERAL(14, 163, 14),
QT_MOC_LITERAL(15, 178, 17),
QT_MOC_LITERAL(16, 196, 15),
QT_MOC_LITERAL(17, 212, 12),
QT_MOC_LITERAL(18, 225, 8),
QT_MOC_LITERAL(19, 234, 6),
QT_MOC_LITERAL(20, 241, 11),
QT_MOC_LITERAL(21, 253, 9),
QT_MOC_LITERAL(22, 263, 11),
QT_MOC_LITERAL(23, 275, 2),
QT_MOC_LITERAL(24, 278, 17),
QT_MOC_LITERAL(25, 296, 4),
QT_MOC_LITERAL(26, 301, 5),
QT_MOC_LITERAL(27, 307, 11),
QT_MOC_LITERAL(28, 319, 9),
QT_MOC_LITERAL(29, 329, 16),
QT_MOC_LITERAL(30, 346, 6),
QT_MOC_LITERAL(31, 353, 10),
QT_MOC_LITERAL(32, 364, 12),
QT_MOC_LITERAL(33, 377, 14)
    },
    "ApplicationWindow\0keyPressEvent\0\0"
    "QKeyEvent*\0event\0timerEvent\0QTimerEvent*\0"
    "changeSlider\0changeDial\0cButtonPress\0"
    "lButtonPress\0rButtonPress\0uButtonPress\0"
    "dButtonPress\0povButtonPress\0"
    "cameraButtonPress\0cameraStepPress\0"
    "fileMenuSlot\0QAction*\0action\0updatePower\0"
    "showEvent\0QShowEvent*\0ev\0setFrameRateLabel\0"
    "rate\0frame\0restartFile\0startLive\0"
    "callDumpFunction\0sender\0closeEvent\0"
    "QCloseEvent*\0changeCheckBox\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ApplicationWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,  119,    2, 0x09,
       5,    1,  122,    2, 0x09,
       7,    0,  125,    2, 0x09,
       8,    0,  126,    2, 0x09,
       9,    0,  127,    2, 0x08,
      10,    0,  128,    2, 0x08,
      11,    0,  129,    2, 0x08,
      12,    0,  130,    2, 0x08,
      13,    0,  131,    2, 0x08,
      14,    0,  132,    2, 0x08,
      15,    0,  133,    2, 0x08,
      16,    0,  134,    2, 0x08,
      17,    1,  135,    2, 0x08,
      20,    0,  138,    2, 0x08,
      21,    1,  139,    2, 0x08,
      24,    2,  142,    2, 0x08,
      27,    0,  147,    2, 0x08,
      28,    0,  148,    2, 0x08,
      29,    1,  149,    2, 0x08,
      31,    1,  152,    2, 0x08,
      33,    0,  155,    2, 0x08,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 18,   19,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 22,   23,
    QMetaType::Void, QMetaType::Float, QMetaType::Int,   25,   26,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 18,   30,
    QMetaType::Void, 0x80000000 | 32,    4,
    QMetaType::Void,

       0        // eod
};

void ApplicationWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ApplicationWindow *_t = static_cast<ApplicationWindow *>(_o);
        switch (_id) {
        case 0: _t->keyPressEvent((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 1: _t->timerEvent((*reinterpret_cast< QTimerEvent*(*)>(_a[1]))); break;
        case 2: _t->changeSlider(); break;
        case 3: _t->changeDial(); break;
        case 4: _t->cButtonPress(); break;
        case 5: _t->lButtonPress(); break;
        case 6: _t->rButtonPress(); break;
        case 7: _t->uButtonPress(); break;
        case 8: _t->dButtonPress(); break;
        case 9: _t->povButtonPress(); break;
        case 10: _t->cameraButtonPress(); break;
        case 11: _t->cameraStepPress(); break;
        case 12: _t->fileMenuSlot((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 13: _t->updatePower(); break;
        case 14: _t->showEvent((*reinterpret_cast< QShowEvent*(*)>(_a[1]))); break;
        case 15: _t->setFrameRateLabel((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 16: _t->restartFile(); break;
        case 17: _t->startLive(); break;
        case 18: _t->callDumpFunction((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 19: _t->closeEvent((*reinterpret_cast< QCloseEvent*(*)>(_a[1]))); break;
        case 20: _t->changeCheckBox(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 12:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QAction* >(); break;
            }
            break;
        case 18:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QAction* >(); break;
            }
            break;
        }
    }
}

const QMetaObject ApplicationWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_ApplicationWindow.data,
      qt_meta_data_ApplicationWindow,  qt_static_metacall, 0, 0}
};


const QMetaObject *ApplicationWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ApplicationWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ApplicationWindow.stringdata))
        return static_cast<void*>(const_cast< ApplicationWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int ApplicationWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    }
    return _id;
}
struct qt_meta_stringdata_NoWrapDial_t {
    QByteArrayData data[4];
    char stringdata[26];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_NoWrapDial_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_NoWrapDial_t qt_meta_stringdata_NoWrapDial = {
    {
QT_MOC_LITERAL(0, 0, 10),
QT_MOC_LITERAL(1, 11, 8),
QT_MOC_LITERAL(2, 20, 0),
QT_MOC_LITERAL(3, 21, 3)
    },
    "NoWrapDial\0onAction\0\0val\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NoWrapDial[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x09,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,

       0        // eod
};

void NoWrapDial::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        NoWrapDial *_t = static_cast<NoWrapDial *>(_o);
        switch (_id) {
        case 0: _t->onAction((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject NoWrapDial::staticMetaObject = {
    { &QDial::staticMetaObject, qt_meta_stringdata_NoWrapDial.data,
      qt_meta_data_NoWrapDial,  qt_static_metacall, 0, 0}
};


const QMetaObject *NoWrapDial::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NoWrapDial::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_NoWrapDial.stringdata))
        return static_cast<void*>(const_cast< NoWrapDial*>(this));
    return QDial::qt_metacast(_clname);
}

int NoWrapDial::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDial::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
struct qt_meta_stringdata_DebugWindow_t {
    QByteArrayData data[4];
    char stringdata[28];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    offsetof(qt_meta_stringdata_DebugWindow_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData) \
    )
static const qt_meta_stringdata_DebugWindow_t qt_meta_stringdata_DebugWindow = {
    {
QT_MOC_LITERAL(0, 0, 11),
QT_MOC_LITERAL(1, 12, 6),
QT_MOC_LITERAL(2, 19, 0),
QT_MOC_LITERAL(3, 20, 6)
    },
    "DebugWindow\0accept\0\0reject\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DebugWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x08,
       3,    0,   25,    2, 0x08,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void DebugWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DebugWindow *_t = static_cast<DebugWindow *>(_o);
        switch (_id) {
        case 0: _t->accept(); break;
        case 1: _t->reject(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject DebugWindow::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_DebugWindow.data,
      qt_meta_data_DebugWindow,  qt_static_metacall, 0, 0}
};


const QMetaObject *DebugWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DebugWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DebugWindow.stringdata))
        return static_cast<void*>(const_cast< DebugWindow*>(this));
    return QDialog::qt_metacast(_clname);
}

int DebugWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
