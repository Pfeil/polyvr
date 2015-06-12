#ifndef VRPARTICLES_H_INCLUDED
#define VRPARTICLES_H_INCLUDED

#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGGeoProperties.h>
#include "core/objects/geometry/VRGeometry.h"
#include "core/utils/VRFunction.h"

class btDiscreteDynamicsWorld;

using namespace std;
OSG_BEGIN_NAMESPACE;

class VRGeometry;
struct Particle;

class VRParticles : public VRGeometry {
    private:
        VRFunction<int>* fkt = 0;
        VRMaterial* mat = 0;
        GeoPnt3fPropertyRecPtr pos;
        int N = 1000;

        btDiscreteDynamicsWorld* world = 0;
        vector<Particle*> particles;

    public:
        VRParticles();
        ~VRParticles();

        void update(int b = 0, int e = -1);
};

OSG_END_NAMESPACE;

#endif // VRPARTICLES_H_INCLUDED