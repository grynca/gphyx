#ifndef CONTACT_H
#define CONTACT_H

#include "../gphyx_domain.h"
#include "../ManifoldCache.h"

namespace grynca {

    template <u32 Size>
    class ContactJoint {
    public:
        template <typename BodiesPairHandler>
        void init(f32 ups, BodiesPairHandler& bph, gphyx::SpeedsHashMapT& speeds, u32 spdA_id, u32 spdB_id,
                   const ContactManifold& cm, ManifoldCache* ct_cache);

        void solve(gphyx::SpeedsHashMapT& speeds, bool& small_io);

        Vec2 normal, tangent;
        f32 imA, imB, iiA, iiB;
        f32 r;          // coeff. of restitution
        f32 f;          // coeff. of friction
        u32 sA_id, sB_id;
        ManifoldCache* cache;

        struct CPInfo {
            Vec2 rA, rB;
            f32 mass_normal, mass_tangent;
            f32 bias;
        } cps_info[Size];
    };
}

#include "ContactJoint.inl"
#endif //CONTACT_H
