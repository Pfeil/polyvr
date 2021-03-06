#ifndef VRENTITY_H_INCLUDED
#define VRENTITY_H_INCLUDED

#include "VROntologyUtils.h"
#include "VRConcept.h"

using namespace std;

struct VREntity : public VRNamedID {
    VRConceptPtr concept = 0;
    map<string, vector<VRPropertyPtr> > properties;

    VREntity(string name, VRConceptPtr c = 0);
    static VREntityPtr create(string name, VRConceptPtr c = 0);
    void setConcept(VRConceptPtr c);

    void set(string name, string value);
    void add(string name, string value);
    string toString();

    vector<VRPropertyPtr> getProperties(string name = "");

    vector<string> getAtPath(vector<string> path);
};

#endif
