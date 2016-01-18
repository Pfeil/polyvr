#ifndef VRPARTICLE_H_INCLUDED
#define VRPARTICLE_H_INCLUDED

#define BIT(x) (1<<(x))

#include "core/scene/VRSceneManager.h"
#include "core/scene/VRScene.h"

#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGGeoProperties.h>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>



using namespace std;
OSG_BEGIN_NAMESPACE;

struct Particle {
    float mass = 0.01; // TODO unit is ???
    float radius = 0.01; // unit is meter
    unsigned int age = 0; // current age
    unsigned int lifetime = 0; // max age. 0 means immortal.
    bool setUp = false;

    btRigidBody* body = 0;
    btCollisionShape* shape = 0;
    btDefaultMotionState* motionState = 0;
    int collisionGroup = 1; // init value -- set up in spawnAt()
    int collisionMask = 2; // init value -- set up in spawnAt()

    Particle(btDiscreteDynamicsWorld* world = 0) {}

    ~Particle() {
        VRScenePtr scene = VRSceneManager::getCurrent();
        if (scene && body) { scene->bltWorld()->removeRigidBody(body); }
        if (body) { delete body; }
        if (shape) { delete shape; }
        if (motionState) { delete motionState; }
    }


    void setup(btVector3 v, bool activate) {
        if (setUp) {
            body->activate(activate);
            return;
        }

        setUp = true;
        btTransform t;
        t.setOrigin(btVector3(v.x(),v.y(),v.z()));
        motionState = new btDefaultMotionState(t);

        shape = new btSphereShape(radius);

        btVector3 inertiaVector(0,0,0);
        shape->calculateLocalInertia(mass, inertiaVector);
        btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motionState, shape, inertiaVector );

        body = new btRigidBody(rbInfo);
        //body->setActivationState(ACTIVE_TAG);
        body->activate(activate);
    }

    void spawnAt(btVector3 v, btDiscreteDynamicsWorld* world = 0, bool collideWithSelf = false) {
        if (world == 0) return;
        // TODO if Particles shall be spawned a second time, handle it.
        // TODO most of this should be done in constructor. Only the last line should be here if possible.

        if (!setUp) {
            setup(v, true);
        } else {
           btTransform t;
           t.setOrigin(btVector3(v.x(),v.y(),v.z()));
           body->getMotionState()->setWorldTransform(t);
           body->activate(true);
        }

        collisionGroup = BIT(1); // 0010 = 2^1 --> setColissionMask(1)
        collisionMask = BIT(0); //  0001 = 2^0 --> setCollisionGroup(0)
        if (collideWithSelf) {
            collisionMask |= collisionGroup;
        }
        world->addRigidBody(body, collisionGroup, collisionMask);
    }

    void setActive(bool active = true) {
        body->activate(active);
    }

};

struct SphParticle : public Particle {
    bool sphActive = true;
    // TODO @depricated
    float sphArea = 5 * radius;
    float sphDensity = 0.0;
    float sphPressure = 0.0;
    btVector3 sphPressureForce;
    btVector3 sphViscosityForce;


    SphParticle(btDiscreteDynamicsWorld* world = 0, bool active = true) {
        sphActive = active;
        sphPressureForce.setZero();
        sphViscosityForce.setZero();
    }

    void setActive(bool active = true) {
        sphActive = active;
        body->activate(active);
    }

};
OSG_END_NAMESPACE;
#endif // VRPARTICLE_H_INCLUDED
