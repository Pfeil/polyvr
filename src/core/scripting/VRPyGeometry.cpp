#include "VRPyGeometry.h"
#include "core/objects/material/VRMaterial.h"
#include "core/objects/material/VRVideo.h"
#include "core/utils/toString.h"
#include "core/scene/VRScene.h"
#include "core/scene/VRSceneManager.h"
#include "VRPyBaseT.h"
#include "VRPyMaterial.h"
#include "VRPySelection.h"
#include "VRPyTypeCaster.h"
#include "VRPyPose.h"

#define NO_IMPORT_ARRAY
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include "numpy/ndarraytypes.h"
#include "numpy/ndarrayobject.h"

#include <OpenSG/OSGGeoProperties.h>
#include <OpenSG/OSGGeometry.h>

template<> PyTypeObject VRPyBaseT<OSG::VRGeometry>::type = {
    PyObject_HEAD_INIT(NULL)
    0,                         /*ob_size*/
    "VR.Geometry",             /*tp_name*/
    sizeof(VRPyGeometry),             /*tp_basicsize*/
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
    "VRGeometry binding",           /* tp_doc */
    0,		               /* tp_traverse */
    0,		               /* tp_clear */
    0,		               /* tp_richcompare */
    0,		               /* tp_weaklistoffset */
    0,		               /* tp_iter */
    0,		               /* tp_iternext */
    VRPyGeometry::methods,             /* tp_methods */
    0,             /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)init,      /* tp_init */
    0,                         /* tp_alloc */
    New_VRObjects_ptr,                 /* tp_new */
};

PyMethodDef VRPyGeometry::methods[] = {
    {"setType", (PyCFunction)VRPyGeometry::setType, METH_VARARGS, "set geometry type - setType(type)" },
    {"setTypes", (PyCFunction)VRPyGeometry::setTypes, METH_VARARGS, "set geometry type - setTypes([type1, type2, ..])\n\ttype can be:"
                                                                                                                    "\n\t GL_POINTS"
                                                                                                                    "\n\t GL_LINES, GL_LINE_LOOP, GL_LINE_STRIP"
                                                                                                                    "\n\t GL_TRIANGLES, GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN"
                                                                                                                    "\n\t GL_QUADS, GL_QUAD_STRIP"
                                                                                                                    "\n\t GL_POLYGON" },
    {"setPositions", (PyCFunction)VRPyGeometry::setPositions, METH_VARARGS, "set geometry positions - setPositions(list of (list of [x,y,z]))" },
    {"setNormals", (PyCFunction)VRPyGeometry::setNormals, METH_VARARGS, "set geometry normals - setNormals([[x,y,z], ...])" },
    {"setColors", (PyCFunction)VRPyGeometry::setColors, METH_VARARGS, "set geometry colors - setColors([[x,y,z], ...])" },
    {"setIndices", (PyCFunction)VRPyGeometry::setIndices, METH_VARARGS, "set geometry indices - setIndices(int[])" },
    {"setLengths", (PyCFunction)VRPyGeometry::setLengths, METH_VARARGS, "set geometry lengths - setLengths(int[])" },
    {"setTexCoords", (PyCFunction)VRPyGeometry::setTexCoords, METH_VARARGS, "set geometry texture coordinates - setTexCoords( [[x,y]], int channel = 0, bool fixMapping = false)" },
    {"setTexture", (PyCFunction)VRPyGeometry::setTexture, METH_VARARGS, "set texture from file - setTexture(path)" },
    {"setMaterial", (PyCFunction)VRPyGeometry::setMaterial, METH_VARARGS, "set material" },
    {"getTypes", (PyCFunction)VRPyGeometry::getTypes, METH_NOARGS, "get geometry primitive types - [int t] getTypes()\n\tt = 0 : GL_POINTS"
                                                                                                                    "\n\tt = 1 : GL_LINES"
                                                                                                                    "\n\tt = 2 : GL_LINE_LOOP"
                                                                                                                    "\n\tt = 3 : GL_LINE_STRIP"
                                                                                                                    "\n\tt = 4 : GL_TRIANGLES"
                                                                                                                    "\n\tt = 5 : GL_TRIANGLE_STRIP"
                                                                                                                    "\n\tt = 6 : GL_TRIANGLE_FAN"
                                                                                                                    "\n\tt = 7 : GL_QUADS"
                                                                                                                    "\n\tt = 8 : GL_QUAD_STRIP"
                                                                                                                    "\n\tt = 9 : GL_POLYGON" },
    {"getLengths", (PyCFunction)VRPyGeometry::getLengths, METH_NOARGS, "get geometry lengths" },
    {"getPositions", (PyCFunction)VRPyGeometry::getPositions, METH_NOARGS, "get geometry positions" },
    {"getNormals", (PyCFunction)VRPyGeometry::getNormals, METH_NOARGS, "get geometry normals" },
    {"getColors", (PyCFunction)VRPyGeometry::getColors, METH_NOARGS, "get geometry colors" },
    {"getIndices", (PyCFunction)VRPyGeometry::getIndices, METH_NOARGS, "get geometry indices" },
    {"getTexCoords", (PyCFunction)VRPyGeometry::getTexCoords, METH_NOARGS, "get geometry texture coordinates" },
    {"getMaterial", (PyCFunction)VRPyGeometry::getMaterial, METH_NOARGS, "get material" },
    {"merge", (PyCFunction)VRPyGeometry::merge, METH_VARARGS, "Merge another geometry into this one - merge( geo )" },
    {"remove", (PyCFunction)VRPyGeometry::remove, METH_VARARGS, "Remove a part of the geometry - remove( Selection s )" },
    {"copy", (PyCFunction)VRPyGeometry::copy, METH_VARARGS, "Copy a part of the geometry - geo copy( Selection s )" },
    {"separate", (PyCFunction)VRPyGeometry::separate, METH_VARARGS, "Copy and remove a part of the geometry - geo separate( Selection s )" },
    {"setPrimitive", (PyCFunction)VRPyGeometry::setPrimitive, METH_VARARGS, "Set geometry to primitive" },
    {"setVideo", (PyCFunction)VRPyGeometry::setVideo, METH_VARARGS, "Set video texture - setVideo(path)" },
    {"playVideo", (PyCFunction)VRPyGeometry::playVideo, METH_VARARGS, "Play the video texture from t0 to t1 - playVideo(t0, t1, speed)" },
    {"decimate", (PyCFunction)VRPyGeometry::decimate, METH_VARARGS, "Decimate geometry by collapsing a fraction of edges - decimate(f)" },
    {"setRandomColors", (PyCFunction)VRPyGeometry::setRandomColors, METH_NOARGS, "Set a random color for each vertex" },
    {"removeDoubles", (PyCFunction)VRPyGeometry::removeDoubles, METH_VARARGS, "Remove double vertices" },
    {"updateNormals", (PyCFunction)VRPyGeometry::updateNormals, METH_NOARGS, "Recalculate the normals of the geometry" },
    {"makeUnique", (PyCFunction)VRPyGeometry::makeUnique, METH_NOARGS, "Make the geometry data unique" },
    {"influence", (PyCFunction)VRPyGeometry::influence, METH_VARARGS, "Pass a points and value vector to influence the geometry - influence([points,f3], [values,f3], int power)" },
    {"showGeometricData", (PyCFunction)VRPyGeometry::showGeometricData, METH_VARARGS, "Enable or disable a data layer - showGeometricData(string type, bool)\n layers are: ['Normals']" },
    {"calcSurfaceArea", (PyCFunction)VRPyGeometry::calcSurfaceArea, METH_NOARGS, "Compute and return the total surface area - flt calcSurfaceArea()" },
    {"setPositionalTexCoords", (PyCFunction)VRPyGeometry::setPositionalTexCoords, METH_VARARGS, "Use the positions as texture coordinates - setPositionalTexCoords(float scale)" },
    {"genTexCoords", (PyCFunction)VRPyGeometry::genTexCoords, METH_VARARGS, "Generate the texture coordinates - genTexCoords( str mapping, float scale, int channel, Pose )\n\tmapping: ['CUBE', 'SPHERE']" },
    {"readSharedMemory", (PyCFunction)VRPyGeometry::readSharedMemory, METH_VARARGS, "Read the geometry from shared memory buffers - readSharedMemory( str segment, str object )" },
    {"applyTransformation", (PyCFunction)VRPyGeometry::applyTransformation, METH_VARARGS, "Apply a transformation to the mesh - applyTransformation( pose )" },
    {NULL}  /* Sentinel */
};

/*
Positions get/set, setTypes
*/

int getListDepth(PyObject* o) {
    string tname;
	tname = o->ob_type->tp_name;
	if (tname == "list") if (PyList_Size(o) > 0) return getListDepth(PyList_GetItem(o, 0))+1;
	if (tname == "numpy.ndarray") {
        PyArrayObject* a = (PyArrayObject*)o;
        return PyArray_NDIM(a);
	}
	return 0;
}

template<class T, class t>
void feed2Dnp(PyObject* o, T& vec) { // numpy version
    PyArrayObject* a = (PyArrayObject*)o;

    int N = PyArray_DIMS(a)[0];
    vec->resize(N);

    t v;
    double* f;
    for (int i=0; i<N; i++) {
        for (int j=0; j<3; j++) {
            f = (double*)PyArray_GETPTR2(a, i, j);
            v[j] = *f;
        }
        vec->setValue(v, i);
    }
}

template<class T, class t>
void feed2D(PyObject* o, T& vec) {
    PyObject *pi, *pj;
    t tmp;
    Py_ssize_t N = PyList_Size(o);

    for (Py_ssize_t i=0; i<N; i++) {
        pi = PyList_GetItem(o, i);
        for (Py_ssize_t j=0; j<PyList_Size(pi); j++) {
            pj = PyList_GetItem(pi, j);
            tmp[j] = PyFloat_AsDouble(pj);
        }
        vec->push_back(tmp);
    }
}

template<class T, class t>
void feed2D_v2(PyObject* o, T& vec) {
    PyObject *pi, *pj;
    t tmp;
    Py_ssize_t N = PyList_Size(o);

    for (Py_ssize_t i=0; i<N; i++) {
        pi = PyList_GetItem(o, i);
        for (Py_ssize_t j=0; j<PyList_Size(pi); j++) {
            pj = PyList_GetItem(pi, j);
            tmp[j] = PyFloat_AsDouble(pj);
        }
        vec.push_back(tmp);
    }
}

template<class T>
void feed1Dnp(PyObject* o, T& vec) {
    PyArrayObject* a = (PyArrayObject*)o;
    int N = PyArray_DIMS(a)[0];
    vec->resize(N);

    for (Py_ssize_t i=0; i<N; i++) {
        int* j = (int*)PyArray_GETPTR1(a, i);
        vec->setValue(*j, i);
    }
}

template<class T>
void feed1D(PyObject* o, T& vec) {
    PyObject *pi;
    Py_ssize_t N = PyList_Size(o);

    for (Py_ssize_t i=0; i<N; i++) {
        pi = PyList_GetItem(o, i);
        int j = PyInt_AsLong(pi);
        vec->addValue(j);
    }
}

template<class T, class t>
void feed1D3np(PyObject* o, T& vec) {
    PyArrayObject* a = (PyArrayObject*)o;
    int N = PyArray_DIMS(a)[0];
    for (Py_ssize_t i=0; i<N; i+=3) {
        t tmp;
        float* x = (float*)PyArray_GETPTR1(a, i+0);
        float* y = (float*)PyArray_GETPTR1(a, i+1);
        float* z = (float*)PyArray_GETPTR1(a, i+2);
        tmp[0] = *x;
        tmp[1] = *y;
        tmp[2] = *z;
        vec->addValue(tmp);
    }
}

template<class T, class t>
void feed1D3(PyObject* o, T& vec) {
    Py_ssize_t N = PyList_Size(o);

    for (Py_ssize_t i=0; i<N; i+=3) {
        t tmp;
        tmp[0] = PyFloat_AsDouble( PyList_GetItem(o, i+0) );
        tmp[1] = PyFloat_AsDouble( PyList_GetItem(o, i+1) );
        tmp[2] = PyFloat_AsDouble( PyList_GetItem(o, i+2) );
        vec->addValue(tmp);
    }
}

PyObject* VRPyGeometry::applyTransformation(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    VRPyPose* pose = 0;
    if (!PyArg_ParseTuple(args, "|O", &pose)) return NULL;
    if (pose) self->objPtr->applyTransformation( pose->objPtr );
    else self->objPtr->applyTransformation();
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::genTexCoords(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    const char* c = 0; float scale; int channel; VRPyPose* pose;
    if (!PyArg_ParseTuple(args, "sfiO", (char*)&c, &scale, &channel, &pose)) return NULL;
    string mapping = "CUBE"; if (c) mapping = c;
    self->objPtr->genTexCoords( mapping, scale, channel, pose->objPtr );
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::separate(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    VRPySelection* sel = 0;
    if (!PyArg_ParseTuple(args, "O", &sel)) return NULL;
    return VRPyGeometry::fromSharedPtr( self->objPtr->separateSelection( sel->objPtr ) );
}

PyObject* VRPyGeometry::copy(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    VRPySelection* sel = 0;
    if (!PyArg_ParseTuple(args, "O", &sel)) return NULL;
    return VRPyGeometry::fromSharedPtr( self->objPtr->copySelection( sel->objPtr ) );
}

PyObject* VRPyGeometry::remove(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    VRPySelection* sel = 0;
    if (!PyArg_ParseTuple(args, "O", &sel)) return NULL;
    self->objPtr->removeSelection( sel->objPtr );
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setPositionalTexCoords(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    self->objPtr->setPositionalTexCoords( parseFloat(args) );
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::updateNormals(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    self->objPtr->updateNormals();
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::calcSurfaceArea(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    return PyFloat_FromDouble( self->objPtr->calcSurfaceArea() );
}

PyObject* VRPyGeometry::showGeometricData(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
	PyObject* type;
	int b;
    if (!PyArg_ParseTuple(args, "Oi", &type, &b)) return NULL;
    self->objPtr->showGeometricData( PyString_AsString(type), b);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::influence(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
	PyObject *vP, *vV; int power; float color_coding; float dl_max;
    if (!PyArg_ParseTuple(args, "OOiff", &vP, &vV, &power, &color_coding, &dl_max)) return NULL;
    vector<OSG::Vec3f> pos;
    vector<OSG::Vec3f> vals;
    feed2D_v2<vector<OSG::Vec3f>, OSG::Vec3f>(vP, pos);
    feed2D_v2<vector<OSG::Vec3f>, OSG::Vec3f>(vV, vals);
    self->objPtr->influence(pos, vals, power, color_coding, dl_max);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::merge(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
	VRPyGeometry* geo;
    if (!PyArg_ParseTuple(args, "O", &geo)) return NULL;
    self->objPtr->merge(geo->objPtr);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setRandomColors(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    self->objPtr->setRandomColors();
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::makeUnique(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    self->objPtr->makeUnique();
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::removeDoubles(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    self->objPtr->removeDoubles( parseFloat(args) );
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::decimate(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    self->objPtr->decimate( parseFloat(args) );
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setType(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;

    string stype = parseString(args);

    int type = toGLConst(stype);
    if (type == -1) {
        PyErr_SetString(err, (stype + " is not a valid type").c_str() );
        return NULL;
    }

    OSG::VRGeometryPtr geo = (OSG::VRGeometryPtr) self->objPtr;
    geo->setType(type);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setTypes(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
	PyObject* typeList;
    if (!PyArg_ParseTuple(args, "O", &typeList)) return NULL;

    OSG::VRGeometryPtr geo = (OSG::VRGeometryPtr) self->objPtr;
    OSG::GeoUInt8PropertyRecPtr types = OSG::GeoUInt8Property::create();

	for (int i = 0; i < pySize(typeList); i++) {
		PyObject* pyType = PyList_GetItem(typeList, i);

		string stype = PyString_AsString(pyType);
		int type = toGLConst(stype);
		if (type == -1) {
			PyErr_SetString(err, (stype + " is not a valid type").c_str() );
			return NULL;
		}

		types->addValue(type);
	}

    geo->setTypes(types);
    Py_RETURN_TRUE;
}

/**
 * @brief Sets the vertex positions of the geometry.
 * @param args A list of lists of vertices: [[[x,y,z], [x,y,z], ...], [[x,y,z], ...]]
 * Each of the lists must contain a single type of primitives that corresponds to the
 * types provided through setTypes()
 */

PyObject* VRPyGeometry::setPositions(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    PyObject* vec;
    if (! PyArg_ParseTuple(args, "O", &vec)) return NULL;

	OSG::GeoPnt3fPropertyRecPtr pos = OSG::GeoPnt3fProperty::create();

    int ld = getListDepth(vec);
    string tname = vec->ob_type->tp_name;

    if (ld == 1) {
        if (tname == "numpy.ndarray") feed1D3np<OSG::GeoPnt3fPropertyRecPtr, OSG::Pnt3f>(vec, pos);
        else feed1D3<OSG::GeoPnt3fPropertyRecPtr, OSG::Pnt3f>(vec, pos);
    } else if (ld == 2) {
        if (tname == "numpy.ndarray") feed2Dnp<OSG::GeoPnt3fPropertyRecPtr, OSG::Pnt3f>(vec, pos);
        else feed2D<OSG::GeoPnt3fPropertyRecPtr, OSG::Pnt3f>(vec, pos);
    } else if (ld == 3) {
        for(Py_ssize_t i = 0; i < PyList_Size(vec); i++) {
            PyObject* vecList = PyList_GetItem(vec, i);
            string tname = vecList->ob_type->tp_name;
            if (tname == "numpy.ndarray") feed2Dnp<OSG::GeoPnt3fPropertyRecPtr, OSG::Pnt3f>(vecList, pos);
            else feed2D<OSG::GeoPnt3fPropertyRecPtr, OSG::Pnt3f>(vecList, pos);
        }
    } else {
        string e = "VRPyGeometry::setPositions - bad argument, ld is " + toString(ld);
        PyErr_SetString(err, e.c_str());
        return NULL;
    }

    self->objPtr->setPositions(pos);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setNormals(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    PyObject* vec;
    if (! PyArg_ParseTuple(args, "O", &vec)) return NULL;

    OSG::GeoVec3fPropertyRecPtr norms = OSG::GeoVec3fProperty::create();
    string tname = vec->ob_type->tp_name;
    int ld = getListDepth(vec);

    if (ld == 1) {
        if (tname == "numpy.ndarray") feed1D3np<OSG::GeoVec3fPropertyRecPtr, OSG::Vec3f>( vec, norms);
        else feed1D3<OSG::GeoVec3fPropertyRecPtr, OSG::Vec3f>( vec, norms);
    } else if (ld == 2) {
        if (tname == "numpy.ndarray") feed2Dnp<OSG::GeoVec3fPropertyRecPtr, OSG::Vec3f>( vec, norms);
        else feed2D<OSG::GeoVec3fPropertyRecPtr, OSG::Vec3f>( vec, norms);
    } else {
        string e = "VRPyGeometry::setNormals - bad argument, ld is " + toString(ld);
        PyErr_SetString(err, e.c_str());
        return NULL;
    }

    self->objPtr->setNormals(norms);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setColors(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    PyObject* vec;
    if (! PyArg_ParseTuple(args, "O", &vec)) return NULL;
    OSG::VRGeometryPtr geo = (OSG::VRGeometryPtr) self->objPtr;

    OSG::GeoVec4fPropertyRecPtr cols = OSG::GeoVec4fProperty::create();
    string tname = vec->ob_type->tp_name;
    if (tname == "numpy.ndarray") feed2Dnp<OSG::GeoVec4fPropertyRecPtr, OSG::Vec4f>( vec, cols);
    else feed2D<OSG::GeoVec4fPropertyRecPtr, OSG::Vec4f>( vec, cols);

    geo->setColors(cols, true);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setLengths(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    PyObject* vec;
    if (!PyArg_ParseTuple(args, "O", &vec)) return NULL;
    OSG::VRGeometryPtr geo = (OSG::VRGeometryPtr) self->objPtr;

    OSG::GeoUInt32PropertyRecPtr lens = OSG::GeoUInt32Property::create();
    feed1D<OSG::GeoUInt32PropertyRecPtr>(vec, lens);
    geo->setLengths(lens);

    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setIndices(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    PyObject* vec;
    if (! PyArg_ParseTuple(args, "O", &vec)) return NULL;

    OSG::GeoUInt32PropertyRecPtr inds = OSG::GeoUInt32Property::create();
    string tname = vec->ob_type->tp_name;

    int ld = getListDepth(vec);
    cout << "setIndices ld: " << ld << endl;
    if (ld == 1) {
        cout << "setIndices ld=1\n";
        if (tname == "numpy.ndarray") feed1Dnp<OSG::GeoUInt32PropertyRecPtr>( vec, inds);
        else feed1D<OSG::GeoUInt32PropertyRecPtr>( vec, inds );
        self->objPtr->setIndices(inds, true);
    } else if (ld == 2) {
        cout << "setIndices ld=2\n";
        OSG::GeoUInt32PropertyRecPtr lengths = OSG::GeoUInt32Property::create();
        for(Py_ssize_t i = 0; i < PyList_Size(vec); i++) {
            PyObject* vecList = PyList_GetItem(vec, i);
            feed1D<OSG::GeoUInt32PropertyRecPtr>( vecList, inds );
            lengths->addValue(PyList_Size(vecList));
        }
        self->objPtr->setIndices(inds);
        self->objPtr->setLengths(lengths);
    } else { PyErr_SetString(err, "VRPyGeometry::setIndices - bad argument"); return NULL; }


    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setTexCoords(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    PyObject* vec;
    int channel = 0;
    int doIndexFix = false;
    if (! PyArg_ParseTuple(args, "O|ii", &vec, &channel, &doIndexFix)) return NULL;

    if (pySize(vec) == 0) Py_RETURN_TRUE;
    int vN = pySize(PyList_GetItem(vec,0));

    if (vN == 2) {
        OSG::GeoVec2fPropertyRecPtr tc = OSG::GeoVec2fProperty::create();
        feed2D<OSG::GeoVec2fPropertyRecPtr, OSG::Vec2f>(vec, tc);
        self->objPtr->setTexCoords(tc, channel, doIndexFix);
    }

    if (vN == 3) {
        OSG::GeoVec3fPropertyRecPtr tc = OSG::GeoVec3fProperty::create();
        feed2D<OSG::GeoVec3fPropertyRecPtr, OSG::Vec3f>(vec, tc);
        self->objPtr->setTexCoords(tc, channel, doIndexFix);
    }

    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setTexture(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    PyObject* _path;
    if (! PyArg_ParseTuple(args, "O", &_path)) return NULL;
    OSG::VRGeometryPtr geo = (OSG::VRGeometryPtr) self->objPtr;

    string path = PyString_AsString(_path);

    geo->getMaterial()->setTexture(path);

    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::setMaterial(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    VRPyMaterial* mat;
    if (! PyArg_ParseTuple(args, "O", &mat)) return NULL;
    self->objPtr->setMaterial(mat->objPtr);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::getPositions(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    if (self->objPtr->getMesh() == 0) { PyErr_SetString(err, "VRPyGeometry::getPositions - Mesh is invalid"); return NULL; }

    OSG::GeoVectorProperty* pos = self->objPtr->getMesh()->getPositions();
    if (pos == 0) return PyList_New(0);
    PyObject* res = PyList_New(pos->size());

    for (uint i=0; i<pos->size(); i++) {
        OSG::Vec3f v;
        pos->getValue(v,i);
        PyObject* pv = toPyTuple(v);
        // append to list
        PyList_SetItem(res, i, pv);
    }

    return res;
}

PyObject* VRPyGeometry::getTypes(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    if (self->objPtr->getMesh() == 0) { PyErr_SetString(err, "VRPyGeometry::getNormals - Mesh is invalid"); return NULL; }

    OSG::GeoIntegralProperty* types = self->objPtr->getMesh()->getTypes();
    if (types == 0) return PyList_New(0);
    PyObject* res = PyList_New(types->size());

    for (uint i=0; i<types->size(); i++) {
        int v;
        types->getValue(v,i);
        PyList_SetItem(res, i, PyInt_FromLong(v));
    }

    return res;
}

PyObject* VRPyGeometry::getLengths(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    if (self->objPtr->getMesh() == 0) { PyErr_SetString(err, "VRPyGeometry::getNormals - Mesh is invalid"); return NULL; }

    OSG::GeoIntegralProperty* lengths = self->objPtr->getMesh()->getLengths();
    if (lengths == 0) return PyList_New(0);
    PyObject* res = PyList_New(lengths->size());

    for (uint i=0; i<lengths->size(); i++) {
        int v;
        lengths->getValue(v,i);
        PyList_SetItem(res, i, PyInt_FromLong(v));
    }

    return res;
}

PyObject* VRPyGeometry::getNormals(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    if (self->objPtr->getMesh() == 0) { PyErr_SetString(err, "VRPyGeometry::getNormals - Mesh is invalid"); return NULL; }

    OSG::GeoVectorProperty* pos = self->objPtr->getMesh()->getNormals();
    if (pos == 0) return PyList_New(0);
    PyObject* res = PyList_New(pos->size());

    for (uint i=0; i<pos->size(); i++) {
        OSG::Vec3f v;
        pos->getValue(v,i);
        PyObject* pv = toPyTuple(v);
        PyList_SetItem(res, i, pv);
    }

    return res;
}

PyObject* VRPyGeometry::getColors(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    if (self->objPtr->getMesh() == 0) { PyErr_SetString(err, "VRPyGeometry::getColors - Mesh is invalid"); return NULL; }

    OSG::GeoVectorProperty* pos = self->objPtr->getMesh()->getColors();
    if (pos == 0) return PyList_New(0);
    PyObject* res = PyList_New(pos->size());

    for (uint i=0; i<pos->size(); i++) {
        OSG::Vec3f v;
        pos->getValue(v,i);
        PyObject* pv = toPyTuple(v);
        PyList_SetItem(res, i, pv);
    }

    return res;
}

PyObject* VRPyGeometry::getIndices(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    if (self->objPtr->getMesh() == 0) { PyErr_SetString(err, "VRPyGeometry::getIndices - Mesh is invalid"); return NULL; }

    OSG::GeoIntegralProperty* pos = self->objPtr->getMesh()->getIndices();
    if (pos == 0) return PyList_New(0);
    PyObject* res = PyList_New(pos->size());

    for (uint i=0; i<pos->size(); i++) {
        int v;
        pos->getValue(v,i);
        PyObject* pv = PyInt_FromLong(v);
        PyList_SetItem(res, i, pv);
    }

    return res;
}

PyObject* VRPyGeometry::getTexCoords(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    if (self->objPtr->getMesh() == 0) { PyErr_SetString(err, "VRPyGeometry::getTexCoords - Mesh is invalid"); return NULL; }

    OSG::GeoVectorProperty* tc = self->objPtr->getMesh()->getTexCoords();
    if (tc == 0) return PyList_New(0);
    PyObject* res = PyList_New(tc->size());

    for (unsigned int i=0; i<tc->size(); i++) {
        OSG::Vec2f v;
        tc->getValue(v,i);
        PyObject* pv = toPyTuple(v);
        PyList_SetItem(res, i, pv);
    }

    return res;
}

PyObject* VRPyGeometry::setVideo(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    OSG::VRGeometryPtr geo = (OSG::VRGeometryPtr) self->objPtr;

    string path = parseString(args);

    geo->getMaterial()->setVideo(path);

    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::playVideo(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    OSG::VRGeometryPtr geo = (OSG::VRGeometryPtr) self->objPtr;

    OSG::Vec3f params = parseVec3f(args);

    geo->getMaterial()->getVideo()->play(0, params[0], params[1], params[2]);

    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::getMaterial(VRPyGeometry* self) {
    if (!self->valid()) return NULL;
    return VRPyMaterial::fromSharedPtr(self->objPtr->getMaterial());
}

PyObject* VRPyGeometry::setPrimitive(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    string params = parseString(args);
    string p1, p2;
    stringstream ss(params);
    ss >> p1; getline(ss, p2);
    self->objPtr->setPrimitive(p1, p2);
    Py_RETURN_TRUE;
}

PyObject* VRPyGeometry::readSharedMemory(VRPyGeometry* self, PyObject *args) {
    if (!self->valid()) return NULL;
    const char* segment = 0;
    const char* object = 0;
    if (! PyArg_ParseTuple(args, "ss", (char*)&segment, (char*)&object)) return NULL;
    if (!segment || !object) Py_RETURN_FALSE;
    self->objPtr->readSharedMemory(segment, object);
    Py_RETURN_TRUE;
}
