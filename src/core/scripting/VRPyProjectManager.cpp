#include "VRPyProjectManager.h"
#include "VRPyObject.h"
#include "VRPyTypeCaster.h"
#include "VRPyBaseT.h"

using namespace OSG;

simpleVRPyType(Storage, 0);
simpleVRPyType(ProjectManager, New_ptr);

PyMethodDef VRPyStorage::methods[] = {
    {NULL}  /* Sentinel */
};

PyMethodDef VRPyProjectManager::methods[] = {
    {"addItem", (PyCFunction)VRPyProjectManager::addItem, METH_VARARGS, "Add a storable item - addItem( i, str mode )\n\tmode can be 'RELOAD' or 'REBUILD', reload will only reload the attributes of the object" },
    {"getItems", (PyCFunction)VRPyProjectManager::getItems, METH_NOARGS, "Get all items - [items] getItems()" },
    {"save", (PyCFunction)VRPyProjectManager::save, METH_VARARGS, "Save to file - save( str path )" },
    {"load", (PyCFunction)VRPyProjectManager::load, METH_VARARGS, "Load from file - load( str path )" },
    {NULL}  /* Sentinel */
};

PyObject* VRPyProjectManager::getItems(VRPyProjectManager* self) {
    if (!self->valid()) return NULL;
    auto objs = self->objPtr->getItems();
    PyObject* li = PyList_New(objs.size());
    for (uint i=0; i<objs.size(); i++) {
        VRObjectPtr o = static_pointer_cast<VRObject>(objs[i]);
        if (o) PyList_SetItem(li, i, VRPyTypeCaster::cast( o ));
    }
    return li;
}

PyObject* VRPyProjectManager::addItem(VRPyProjectManager* self, PyObject* args) {
    if (!self->valid()) return NULL;
    PyObject* o; const char* m = 0;
    if (! PyArg_ParseTuple(args, "Os", &o, (char*)&m)) return NULL;
    if (!m) return NULL;

    VRStoragePtr s;
    if (VRPyObject::check(o)) s = dynamic_pointer_cast<VRStorage>(((VRPyObject*)o)->objPtr);
    else if (VRPyStorage::check(o)) s = ((VRPyStorage*)o)->objPtr;
    else { PyErr_SetString(err, "Not a storable item!"); return NULL; }

    self->objPtr->addItem(s, m);
    Py_RETURN_TRUE;
}

PyObject* VRPyProjectManager::save(VRPyProjectManager* self, PyObject* args) {
    if (!self->valid()) return NULL;
    const char* p;
    if (! PyArg_ParseTuple(args, "s", (char*)&p)) return NULL;
    if (p) self->objPtr->save(p);
    Py_RETURN_TRUE;
}

PyObject* VRPyProjectManager::load(VRPyProjectManager* self, PyObject* args) {
    if (!self->valid()) return NULL;
    const char* p;
    if (! PyArg_ParseTuple(args, "s", (char*)&p)) return NULL;
    if (p) self->objPtr->load(p);
    Py_RETURN_TRUE;
}




