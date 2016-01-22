#ifndef VRFLUIDS_H_INCLUDED
#define VRFLUIDS_H_INCLUDED

#include "../Particles/VRParticles.h"

OSG_BEGIN_NAMESPACE;

class VRFluids : public VRParticles {

    public:
        enum SimulationType { SPH, XSPH };

        VRFluids();
        VRFluids(bool spawnParticles);
        ~VRFluids();
        static shared_ptr<VRFluids> create();

        void update(int from, int to) override;
        void updateSPH(int from, int to);
        void updateXSPH(int from, int to);

        void setSimulation(SimulationType t, bool forceChange=false);
        void setSphRadius(float newRadius);
        void setViscosity(float factor);
        void setMass(float newMass, float variation=0.0) override;

    protected:
        VRUpdatePtr fluidFkt;
        SimulationType simulation = SPH;

        /* Calculate after bullets physics cycle? */
        const bool afterBullet = false;
        /*
         * Pressure multiplier, derived from ideal gas law
         * P = N*(gas const)*(temperature) * (1/V)
         */
        float PRESSURE_KAPPA = 0; // just some init value, see updateDerivedValues()
        /* Number of particles around a resting particle */
        const int REST_N = 1;
        /* Average distance of particles around resting particle */
        const float REST_DIS = 0.7;
        /*
         * Density where particles should rest.
         * (re-)calculate using updateDerivedValues();
         */
        float REST_DENSITY  = 1; // just some init value, see updateDerivedValues()
        /* Simple viscosity multiplier */
        float VISCOSITY_MU   = 0.01;
        /* The number Pi given precisely to five decimal places */
        const float Pi = 3.14159;
        /* The average sph radius of a particle. */
        float sphRadius = 1;
        /* The average mass of a particle. */
        float particleMass = 1;
        /* The average volume of a particle */
        float particleVolume = 1;

        double perform_time = 0.0;
        int perform_iter = 0;
        double perform_octree_insert = 0.0;
        double perform_octree_search = 0.0;
        double perform_calc_props = 0.0;
        double perform_calc_forces = 0.0;

        inline void xsph_calc_movement(SphParticle* p, int from, int to);

        inline float kernel_poly6(btVector3 distance_vector, float area);
        inline float kernel_spiky(btVector3 distance_vector, float area);
        inline btVector3 kernel_spiky_gradient(btVector3 distance_vector, float h);
        inline float kernel_visc(btVector3 distance_vector, float area);
        inline float kernel_visc_laplacian(btVector3 distance_vector, float area);

        inline void sph_calc_properties(SphParticle* p);
        inline void sph_calc_forces(SphParticle* p);

        void setFunctions(int from, int to) override;
        void disableFunctions() override;
        void updateDerivedValues();
};

OSG_END_NAMESPACE;

#endif // VRFLUIDS_H_INCLUDED
