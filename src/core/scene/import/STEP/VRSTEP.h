#ifndef VRSTEP_H_INCLUDED
#define VRSTEP_H_INCLUDED

class STEPfile;
class Registry;
class InstMgr;
class SDAI_Application_instance;
typedef SDAI_Application_instance STEPentity;
class STEPaggregate;
class SDAI_Select;
class STEPattribute;
class STEPcomplex;

#include <memory>
#include <string>
#include <map>
#include <tuple>
#include <OpenSG/OSGVector.h>
#include <OpenSG/OSGLine.h>
#include "core/objects/VRObjectFwd.h"
#include "core/utils/VRFunctionFwd.h"
#include "core/math/pose.h"

OSG_BEGIN_NAMESPACE;
using namespace std;

class VRSTEP {
    public:
        typedef shared_ptr<Registry> RegistryPtr;
        typedef shared_ptr<InstMgr> InstMgrPtr;
        typedef shared_ptr<STEPfile> STEPfilePtr;

        struct Type {
            bool print = false;
            string path; // args
            shared_ptr< VRFunction<STEPentity*> > cb;
        };

        struct Instance {
            string type;
            STEPentity* entity = 0;
            int ID = -1;
            void* data = 0;
            template<size_t i, class... Args>
            typename std::tuple_element<i, tuple<Args...> >::type get() {
                auto t = (tuple<Args...>*)data;
                return std::get<i>(*t);
            }

            operator bool() const { return data != 0; }
        };

        struct Edge;
        struct Bound;
        struct Surface;

    public:
        RegistryPtr registry;
        InstMgrPtr instMgr;
        STEPfilePtr sfile;

        map<string, bool> blacklist;
        int blacklisted = 0;

        string redBeg  = "\033[0;38;2;255;150;150m";
        string greenBeg  = "\033[0;38;2;150;255;150m";
        string blueBeg  = "\033[0;38;2;150;150;255m";
        string colEnd = "\033[0m";

        map<STEPentity*, Instance> instances;
        map<int, Instance> instancesById;
        map<string, vector<Instance> > instancesByType;
        map<STEPentity*, VRTransformPtr> resGeos;
        VRTransformPtr resRoot;

        map<string, Type> types;
        Instance& getInstance(STEPentity* e);
        template<class T> void addType(string type, string path, bool print = false);
        template<class T> void parse(STEPentity* e, string path, string type);
        template<typename T> bool query(STEPentity* e, string path, T& t);
        STEPentity* getSelectEntity(SDAI_Select* s, string ID);

        void loadT(string file, STEPfilePtr sfile, bool* done);
        void open(string file);
        string indent(int lvl);

        void traverseEntity(STEPentity* se, int lvl, STEPcomplex* cparent = 0);
        void traverseSelect(SDAI_Select* s, string ID, int lvl);
        void traverseAggregate(STEPaggregate* sa, int type, int lvl);

        void buildGeometries();
        void buildScenegraph();
        void build();

    public:
        VRSTEP();

        void load(string file, VRTransformPtr res);
};

OSG_END_NAMESPACE;


#endif // VRSTEP_H_INCLUDED
