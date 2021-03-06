#include "VRLightBeacon.h"
#include "VRLight.h"
#include "core/objects/material/VRMaterial.h"
#include "core/utils/VRStorage_template.h"
#include <OpenSG/OSGMultiPassMaterial.h>
#include <OpenSG/OSGSimpleMaterial.h>
#include <OpenSG/OSGSimpleGeometry.h>        // Methods to create simple geos.
#include <libxml++/nodes/element.h>

OSG_BEGIN_NAMESPACE;
using namespace std;

VRMaterialPtr getLightGeoMat() {
    static VRMaterialPtr mat = 0;
    if (mat == 0) {
        mat = VRMaterial::create("light_geo_mat");
        mat->setAmbient(Color3f(0.7, 0.7, 0.7));
        mat->setDiffuse(Color3f(0.9, 0.9, 0.9));
        mat->setSpecular(Color3f(0.4, 0.4, 0.4));
        mat->setTransparency(0.3);
        mat->setLit(false);
        //mat->setDiffuse(Color4f(0,0,1,1));
    }
    return mat;
}

VRLightBeacon::VRLightBeacon(string name) : VRTransform(name) {
    type = "LightBeacon";
    lightGeo = 0;

    GeometryMTRecPtr lightGeo_ = makeSphereGeo(2,0.1);
    lightGeo = makeNodeFor(lightGeo_);
    lightGeo->setTravMask(0);
    lightGeo_->setMaterial(getLightGeoMat()->getMaterial());
    addChild(lightGeo);

    storeObjName("light", &light, &light_name);
}

VRLightBeacon::~VRLightBeacon() {}

VRLightBeaconPtr VRLightBeacon::ptr() { return static_pointer_cast<VRLightBeacon>( shared_from_this() ); }
VRLightBeaconPtr VRLightBeacon::create(string name) {
    auto p = shared_ptr<VRLightBeacon>(new VRLightBeacon(name) );
    getAll().push_back( p );
    return p;
}

void VRLightBeacon::showLightGeo(bool b) {
    if (b) lightGeo->setTravMask(0xffffffff);
    else lightGeo->setTravMask(0);
}

VRLightWeakPtr VRLightBeacon::getLight() { return light; }
void VRLightBeacon::setLight(VRLightPtr l) { light = l; }

vector<VRLightBeaconPtr>& VRLightBeacon::getAll() {
    static vector<VRLightBeaconPtr> objs;
    return objs;
}

OSG_END_NAMESPACE;
