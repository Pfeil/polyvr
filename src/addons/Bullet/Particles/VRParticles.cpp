#include "VRParticles.h"
#include "core/objects/material/VRMaterial.h"
#include "core/scene/VRSceneManager.h"
#include "core/scene/VRScene.h"

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

typedef boost::recursive_mutex::scoped_lock BLock;

using namespace std;
using namespace OSG;

boost::recursive_mutex& mtx() {
    auto scene = OSG::VRSceneManager::getCurrent();
    if (scene) return scene->physicsMutex();
    else {
        static boost::recursive_mutex m;
        return m;
    };
}

struct OSG::Particle {
    float mass = 1.0; // TODO unit is ???
    float radius = 0.01; // unit is meter
    unsigned int age = 0; // current age
    unsigned int lifetime = 0; // max age. 0 means immortal.

    btRigidBody* body = 0;
    btCollisionShape* shape = 0;
    btDefaultMotionState* motionState = 0;
    int collisionGroup = 1;
    int collisionMask = 1;

    void spawnAt(btVector3 v, btDiscreteDynamicsWorld* world = 0) {
        if (world == 0) return;

        btTransform t;
        t.setOrigin(btVector3(v.x(),v.y(),v.z()));
        motionState = new btDefaultMotionState(t);

        shape = new btSphereShape(radius);

        btVector3 inertiaVector(0,0,0);
        shape->calculateLocalInertia(mass, inertiaVector);
        btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motionState, shape, inertiaVector );

        body = new btRigidBody(rbInfo);
        body->setActivationState(ACTIVE_TAG);

        world->addRigidBody(body, collisionGroup, collisionMask);
    }

    Particle(btDiscreteDynamicsWorld* world = 0) {}

    ~Particle() {
        VRScene* scene = VRSceneManager::getCurrent();
        if (scene) scene->bltWorld()->removeRigidBody(body);
        delete body;
        delete shape;
        delete motionState;
    }
};

Vec3f toVec3f(btVector3 v) { return Vec3f(v[0], v[1], v[2]); }
btVector3 toBtVector3(Vec3f v) { return btVector3(v[0], v[1], v[2]); }

VRParticles::VRParticles(int particleAmount) : VRGeometry("particles") {
    N = particleAmount;
    particles.resize(N, 0);

    // physics
    // VRScene* scene = VRSceneManager::getCurrent();
    // if (scene) world = scene->bltWorld();

    {
        BLock lock(mtx());
        for(int i=0;i<N;i++) particles[i] = new Particle();
    }

    // material
    mat = new VRMaterial("particles");
    mat->setDiffuse(Vec3f(0,0,1));
    mat->setPointSize(5);
    mat->setLit(false);

    // geometry
    GeoUInt32PropertyRecPtr     Length = GeoUInt32Property::create();
    GeoUInt32PropertyRecPtr     inds = GeoUInt32Property::create();
    pos = GeoPnt3fProperty::create();

    Length->addValue(N);

    for(int i=0;i<N;i++) pos->addValue(Pnt3f(0,0,0));
    for(int i=0;i<N;i++) inds->addValue(i);

    setType(GL_POINTS);
    setLengths(Length);
    setPositions(pos);
    setIndices(inds);
    setMaterial(mat);

    // update loop
    fkt = new VRFunction<int>("particles_update", boost::bind(&VRParticles::update, this,0,-1));
}

void VRParticles::spawnCube() {
    VRScene* scene = VRSceneManager::getCurrent();
    if (scene) world = scene->bltWorld();
    // spawn
    btVector3 v;
    int i;
    for (i=0;i<N;i++) {
        v.setX (0.1*float(rand())/RAND_MAX);
        v.setY (0.1*float(rand())/RAND_MAX);
        v.setZ (0.1*float(rand())/RAND_MAX);
        particles[i]->spawnAt(v, world);
    }
    // activate update loop
    scene->addUpdateFkt(fkt);
}

VRParticles::~VRParticles() {
    VRScene* scene = VRSceneManager::getCurrent();
    if (scene) scene->dropUpdateFkt(fkt);
    delete mat;

    {
        BLock lock(mtx());
        for (int i=0;i<N;i++) delete particles[i];
    }
}

void VRParticles::update(int b, int e) {
    if (e < 0) e = N;
    {
        BLock lock(mtx());
        for (int i=b; i < e; i++) {
            auto p = particles[i]->body->getWorldTransform().getOrigin();
            pos->setValue(toVec3f(p),i);
        }
    }
    setPositions(pos);
}
