/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "specificworker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SpecificWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x0a,
      33,   26,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SpecificWorker[] = {
    "SpecificWorker\0\0compute()\0myPick\0"
    "setPick(Pick)\0"
};

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SpecificWorker *_t = static_cast<SpecificWorker *>(_o);
        switch (_id) {
        case 0: _t->compute(); break;
        case 1: _t->setPick((*reinterpret_cast< const Pick(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SpecificWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SpecificWorker::staticMetaObject = {
    { &GenericWorker::staticMetaObject, qt_meta_stringdata_SpecificWorker,
      qt_meta_data_SpecificWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SpecificWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker))
        return static_cast<void*>(const_cast< SpecificWorker*>(this));
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
