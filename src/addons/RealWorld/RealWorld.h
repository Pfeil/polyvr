#ifndef REALWORLD_H
#define REALWORLD_H
/*
    TODO:
        - look into splitting
            https://github.com/MaZderMind/osm-history-splitter
            http://wiki.openstreetmap.org/wiki/Osmconvert#Clipping_based_on_Longitude_and_Latitude
*/

#include "core/objects/VRObjectFwd.h"
#include "Altitude.h"

class OSMMapDB;

OSG_BEGIN_NAMESPACE;
using namespace std;

class TrafficSimulation;
class MapCoordinator;
class World;
class MapManager;

class RealWorld {
    private:
        MapCoordinator* mapCoordinator = 0;
        World* world = 0;
        MapManager* mapManager = 0;
        OSMMapDB* mapDB = 0;
        TrafficSimulation* trafficSimulation = 0; // Needed for script access
        static Altitude altitude; // constructor runs once, single instance
        static RealWorld* singelton;

        bool physicalized = false;

    public:
        RealWorld(OSG::VRObjectPtr root);
        ~RealWorld();

        static RealWorld* get();

        void enableModule(string mod, bool b);
        void update(OSG::Vec3f pos);
        void physicalize(bool b);

        TrafficSimulation* getTrafficSimulation();
        MapCoordinator* getCoordinator();
        MapManager* getManager();
        World* getWorld();
        OSMMapDB* getDB();
};

OSG_END_NAMESPACE;

#endif // REALWORLD_H
