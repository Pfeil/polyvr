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
        int N = 200;

        btDiscreteDynamicsWorld* world = 0;
        vector<Particle*> particles;

    public:
        VRParticles(): VRParticles(200){}
        VRParticles(int particleAmount);
        ~VRParticles();

        void setGroupMaxSize(unsigned int maximum);
        int getGroupMaxSize();
        void setDamping(float linear, float angular);


        void formCuboidAt(float posx, float posy, float posz, float sx, float sy, float sz);
        void applyCentralImpulse(float x, float y, float z);

        void update(int b = 0, int e = -1);
};

OSG_END_NAMESPACE;

#endif // VRPARTICLES_H_INCLUDED
