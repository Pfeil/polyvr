#include "VRPyParticles.h"
#include "core/scripting/VRPyTransform.h"
#include "core/scripting/VRPyGeometry.h"
#include "core/scripting/VRPyBaseT.h"


template<> PyTypeObject VRPyBaseT<OSG::VRParticles>::type = {
    PyObject_HEAD_INIT(NULL)
    0,                         /*ob_size*/
    "VR.Particles",             /*tp_name*/
    sizeof(VRPyParticles),             /*tp_basicsize*/
    0,                         /*tp_itemsize*/
    (destructor)dealloc, /*tp_dealloc*/
    0,                         /*tp_print*/
    0,                         /*tp_getattr*/
    0,                         /*tp_setattr*/
    0,                         /*tp_compare*/
    0,                         /*tp_repr*/
    0,                         /*tp_as_number*/
    0,                         /*tp_as_sequence*/
    0,                         /*tp_as_mapping*/
    0,                         /*tp_hash */
    0,                         /*tp_call*/
    0,                         /*tp_str*/
    0,                         /*tp_getattro*/
    0,                         /*tp_setattro*/
    0,                         /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /*tp_flags*/
    "VRParticles binding",           /* tp_doc */
    0,		               /* tp_traverse */
    0,		               /* tp_clear */
    0,		               /* tp_richcompare */
    0,		               /* tp_weaklistoffset */
    0,		               /* tp_iter */
    0,		               /* tp_iternext */
    VRPyParticles::methods,             /* tp_methods */
    0,             /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)init,      /* tp_init */
    0,                         /* tp_alloc */
    New_VRObjects_unnamed,                 /* tp_new */
};

PyMethodDef VRPyParticles::methods[] = {
    {"getGeometry", (PyCFunction)VRPyParticles::getGeometry, METH_VARARGS, "Get geometry - Geometry getGeometry()" },
    //{"applyCentralImpulse", (PyCFunction)VRPyParticles::applyCentralImpulse, METH_VARARGS, "Set a short impulse on every particle in a single direction." },
    {NULL}  /* Sentinel */
};

PyObject* VRPyParticles::getGeometry(VRPyParticles* self) {
    Py_RETURN_TRUE;
}
/*
PyObject* applyCentralImpulse(VRPyParticles* self, PyObject* args) {

    OSG::Vec3f input = VRPyBase::parseVec3f(args);

    if (self->obj == 0) self->obj = new OSG::VRParticles();
    self->obj->applyCentralImpulse(input);
    Py_RETURN_TRUE;
}
*/
