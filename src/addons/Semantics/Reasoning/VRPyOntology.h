#ifndef VRPYONTOLOGY_H_INCLUDED
#define VRPYONTOLOGY_H_INCLUDED

#include "VROntology.h"
#include "core/scripting/VRPyBase.h"

struct VRPyProperty : VRPyBaseT<VRProperty> {
    static PyMethodDef methods[];
    static PyObject* toString(VRPyProperty* self);
    static PyObject* getName(VRPyProperty* self);
    static PyObject* getType(VRPyProperty* self);
    static PyObject* getValue(VRPyProperty* self);
};

struct VRPyConcept : VRPyBaseT<VRConcept> {
    static PyMethodDef methods[];
    static PyObject* toString(VRPyConcept* self);
    static PyObject* getName(VRPyConcept* self);
    static PyObject* getProperty(VRPyConcept* self, PyObject* args);
    static PyObject* getProperties(VRPyConcept* self);
    static PyObject* addProperty(VRPyConcept* self, PyObject* args);
};

struct VRPyOntologyRule : VRPyBaseT<VROntologyRule> {
    static PyMethodDef methods[];
    static PyObject* toString(VRPyOntologyRule* self);
};

struct VRPyEntity : VRPyBaseT<VREntity> {
    static PyMethodDef methods[];
    static PyObject* toString(VRPyEntity* self);
    static PyObject* getName(VRPyEntity* self);
    static PyObject* getConcept(VRPyEntity* self);
    static PyObject* getProperties(VRPyEntity* self, PyObject* args);
    static PyObject* set(VRPyEntity* self, PyObject* args);
    static PyObject* add(VRPyEntity* self, PyObject* args);
};

struct VRPyOntology : VRPyBaseT<VROntology> {
    static PyMethodDef methods[];
    static PyObject* open(VRPyOntology* self, PyObject* args);
    static PyObject* toString(VRPyOntology* self);
    static PyObject* getConcept(VRPyOntology* self, PyObject* args);
    static PyObject* getConcepts(VRPyOntology* self);
    static PyObject* getEntities(VRPyOntology* self, PyObject* args);
    static PyObject* addConcept(VRPyOntology* self, PyObject* args);
    static PyObject* addEntity(VRPyOntology* self, PyObject* args);
    static PyObject* addRule(VRPyOntology* self, PyObject* args);
    static PyObject* merge(VRPyOntology* self, PyObject* args);
    static PyObject* copy(VRPyOntology* self);
};

#endif // VRPYONTOLOGY_H_INCLUDED
