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
    float mass = 1.0;           // unit is ???
    float radius = 0.01;        // unit is meter
    unsigned int age = 0;
    unsigned int lifetime = 0;  // 0 means immortal

    btRigidBody* body = 0;
    btCollisionShape* shape = 0;
    btDefaultMotionState* motionState = 0;
    int collisionGroup = 1;
    int collisionMask = 1;


    Particle(btDiscreteDynamicsWorld* world = 0, unsigned int lifetime = 0) {
        if (world == 0) return;

        this->lifetime = lifetime;

        float x = 0.4*float(rand())/RAND_MAX;
        float y = 0.4*float(rand())/RAND_MAX;
        float z = 0.4*float(rand())/RAND_MAX;

        btTransform t;
        t.setOrigin(btVector3(x,y,z));
        motionState = new btDefaultMotionState(t);

        shape = new btSphereShape(radius);

        btVector3 inertiaVector(0,0,0);
        shape->calculateLocalInertia(mass, inertiaVector);
        btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, motionState, shape, inertiaVector );

        body = new btRigidBody(rbInfo);
        body->setActivationState(ACTIVE_TAG);

        world->addRigidBody(body, collisionGroup, collisionMask);
    }

    ~Particle() {
        VRScene* scene = VRSceneManager::getCurrent();
        if (scene) scene->bltWorld()->removeRigidBody(body);
        delete body;
        delete shape;
        delete motionState;
    }

    void setPosition(float x, float y, float z) {
        btTransform t;
        t.setOrigin(btVector3(x,y,z));
        this->motionState->setWorldTransform(t);
    }
};

struct Emitter {
    vector<int> position = {0,0,0};         // probably obsolete since bullet will handle this
    vector<int> direction = {1,0,0};        // may be obsolete?
    vector<int> area = {1,1};               // area of emission
    vector<float> velocity_range = {0.1,1}; // particle velocity
    unsigned int age = 0;
    unsigned int lifetime = 0;              // 0 means immortal
    vector<int> frequency_range = {1,3};    // range of frequency

    Emitter(btDiscreteDynamicsWorld* world = 0) {
        if (world == 0) return;

        Particle p(world);
    }
};

inline Vec3f toVec3f(btVector3 v) { return Vec3f(v[0], v[1], v[2]); }
inline btVector3 toBtVector3(Vec3f v) { return btVector3(v[0], v[1], v[2]); }

VRParticles::VRParticles(int particleAmount) : VRGeometry("particles") {
    //N = 200;
    //particles.resize(N, 0);
    setGroupMaxSize(particleAmount);

    // physics
    VRScene* scene = VRSceneManager::getCurrent();
    if (scene) world = scene->bltWorld();

    {
        BLock lock(mtx());
        for(int i=0;i<N;i++) particles[i] = new Particle(world, 1000);
    }
    this->setDamping(2, 0.5);
    this->applyCentralImpulse(-0.5,3,-0.5);

    // material
    mat = new VRMaterial("particles");
    mat->setDiffuse(Vec3f(0,0,1));
    mat->setPointSize(5);
    mat->setLit(false);

    // geometry
    GeoUInt32PropertyRecPtr     length = GeoUInt32Property::create();
    GeoUInt32PropertyRecPtr     inds = GeoUInt32Property::create();
    pos = GeoPnt3fProperty::create();

    length->addValue(N);

    for(int i=0;i<N;i++) pos->addValue(Pnt3f(0,0,0));
    for(int i=0;i<N;i++) inds->addValue(i);

    setType(GL_POINTS);
    setLengths(length);
    setPositions(pos);
    setIndices(inds);
    setMaterial(mat);

    // update loop
    fkt = new VRFunction<int>("particles_update", boost::bind(&VRParticles::update, this,0,-1));
    scene->addUpdateFkt(fkt);
}

VRParticles::~VRParticles() {
    VRScene* scene = VRSceneManager::getCurrent();
    if (scene) scene->dropUpdateFkt(fkt);
    delete mat;

    BLock lock(mtx());
    for (int i=0;i<N;i++) delete particles[i];
}

void VRParticles::formCuboidAt(float posx, float posy, float posz, float sx, float sy, float sz) {
    int i;
    for (i=0; i<N; i++) {
        float x = posx+ sx*float(rand())/RAND_MAX;
        float y = posy+ sy*float(rand())/RAND_MAX;
        float z = posz+ sz*float(rand())/RAND_MAX;
        particles[i]->setPosition(x,y,z);
    }
}

void VRParticles::applyCentralImpulse(float x, float y, float z) {
    btVector3 v = {x,y,z};

    int i;
    for (i=0; i<N; i++) {
        Particle *p = particles[i];
        p->body->applyCentralImpulse(v);
    }
}

void VRParticles::update(int b, int e) {
    if (e < 0) e = N;
    {
        BLock lock(mtx());
        for (int i=b; i < e; i++) {
            auto o = particles[i]->body->getWorldTransform().getOrigin();
            pos->setValue(toVec3f(o),i);
            if (particles[i]->lifetime >0 && ++particles[i]->age >= particles[i]->lifetime) {
                //cout << "TODO: reuse Particle " << i << endl; //TODO manage dying particles
                particles[i]->body->setActivationState(DISABLE_SIMULATION);
            }
        }
    }
    setPositions(pos);
}

void VRParticles::setGroupMaxSize(unsigned int maximum) {
    particles.resize(maximum, 0);
    N = maximum;
}

int VRParticles::getGroupMaxSize() { return this->N; }

void VRParticles::setDamping(float linear, float angular) {
    for (int i=0; i<N; i++) {
        //particles[i]->body->setDamping(linear, angular);
        particles[i]->body->setFriction(linear);
    }
}

/*void VRParticles::emitCircle(btVector3 pos, btVector3 dir, float angle, float jitter) {
    Particle* p = particles[i];
    p->setPosition()
    for (int i=0; i<N; i++) {
        particles[i]->setPosition(pos[0], pos[1], pos[2]);
        float pAngle = angle + jitter*float(rand())/RAND_MAX;
        //TODO transform dir with angle in a random direction
        //btTransform t = new btTransform();
        particles[i]->applyCentralImpulse(dir);
    }
}*/
