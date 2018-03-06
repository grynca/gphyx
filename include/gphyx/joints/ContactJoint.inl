#include "ContactJoint.h"
#include "../gphyx_config.h"

#define CJ_TPL template <u32 Size>
#define CJ_TYPE ContactJoint<Size>

namespace grynca {
    CJ_TPL
    template <typename BodiesPairHandler>
    inline void CJ_TYPE::init(f32 ups, BodiesPairHandler& bph, gphyx::SpeedsHashMapT& speeds, u32 spdA_id, u32 spdB_id,
                                   const ContactManifold& cm, ManifoldCache* ct_cache)
    {
        Body& bA = bph.accBodyA();
        Body& bB = bph.accBodyB();
        Transform& tA = bph.accTransformA();
        Transform& tB = bph.accTransformB();

        r = 1+std::max(bA.getMaterial().restitution, bB.getMaterial().restitution);
        f = sqrtf(bA.getMaterial().friction*bB.getMaterial().friction);

        // make normal pointing out
        normal = -cm.normal;
        tangent = cross(normal, 1.0f);

        imA = bA.getMassData().inv_mass;
        iiA = bA.getMassData().inv_inertia;
        imB = bB.getMassData().inv_mass;
        iiB = bB.getMassData().inv_inertia;
        cache = ct_cache;

        f32 inv_mass_sum = imA + imB;
        for (u32 i=0; i<ARRAY_SIZE(cps_info); ++i) {
            auto& cpi = cps_info[i];

            cpi.rA = cm.points[i].position-tA.getPosition();
            cpi.rB = cm.points[i].position-tB.getPosition();

            // penetration is > 0
            cpi.bias = -gphyx::CONTACTS_BETA*ups*std::min(-cm.points[i].penetration + gphyx::PENETRATION_SLOP, 0.0f);

            // Precompute normal mass, tangent mass, and bias.
            f32 rn1 = dot(cpi.rA, normal);
            f32 rn2 = dot(cpi.rB, normal);
            f32 k_normal = inv_mass_sum + iiA * (dot(cpi.rA, cpi.rA) - rn1 * rn1) + iiB * (dot(cpi.rB, cpi.rB) - rn2 * rn2);
            ASSERT(k_normal != 0.0f);
            cpi.mass_normal = 1.0f / k_normal;

            f32 rt1 = dot(cpi.rA, tangent);
            f32 rt2 = dot(cpi.rB, tangent);
            f32 k_tangent = inv_mass_sum + iiA * (dot(cpi.rA, cpi.rA) - rt1 * rt1) + iiB * (dot(cpi.rB, cpi.rB) - rt2 * rt2);
            ASSERT(k_tangent != 0.0f);
            cpi.mass_tangent = 1.0f/ k_tangent;
        }

        sA_id = spdA_id;
        sB_id = spdB_id;
        if (cm.isCached()) {
            cache->update(cm);

            Speed& sA = speeds.accItem(sA_id);
            Speed& sB = speeds.accItem(sB_id);
            Vec2& ls1 = sA.accLinearSpeed();
            Angle& as1 = sA.accAngularSpeed();
            Vec2& ls2 = sB.accLinearSpeed();
            Angle& as2 = sB.accAngularSpeed();

            for (u32 i=0; i<ARRAY_SIZE(cps_info); ++i) {
                // Apply normal + friction impulse
                // needed for (accumulated impulses)
                auto& cpi = cps_info[i];
                ContactCache& cc = cache->cc[i];
                Vec2 P = normal*cc.normal + tangent*cc.tangent;
                ls1 -= imA * P;
                as1 -= iiA * cross(cpi.rA, P);
                ls2 += imB * P;
                as2 += iiB * cross(cpi.rB, P);
            }
        }
        else {
            cache->clear();
        }
    }

    CJ_TPL
    inline void CJ_TYPE::solve(gphyx::SpeedsHashMapT& speeds, bool& small_io) {
        Speed& sA = speeds.accItem(sA_id);
        Speed& sB = speeds.accItem(sB_id);
        Vec2& ls1 = sA.accLinearSpeed();
        Angle& as1 = sA.accAngularSpeed();
        Vec2& ls2 = sB.accLinearSpeed();
        Angle& as2 = sB.accAngularSpeed();

        f32 imp_n = 0.0f;
        f32 imp_t = 0.0f;

        for (u32 i=0; i<ARRAY_SIZE(cps_info); ++i) {
            auto& cpi = cps_info[i];
            ContactCache& cc = cache->cc[i];
            // rel. velocity at contact
            Vec2 dv = ls2 + cross(as2, cpi.rB) - ls1 - cross(as1, cpi.rA);

            // normal impulse
            f32 vn = dot(dv, normal);
            f32 inc_imp_n = cpi.mass_normal * (cpi.bias - vn*r);

            // clamp
            f32 acc_imp_n_old = cc.normal;
            cc.normal = std::max(acc_imp_n_old + inc_imp_n, 0.0f);
            inc_imp_n = cc.normal - acc_imp_n_old;

            // apply normal impulse
            Vec2 pn = inc_imp_n * normal;
//            ls1 -= imA * pn;
//            as1 -= iiA * cross(cpi.rA, pn);
//            ls2 += imB * pn;
//            as2 += iiB * cross(cpi.rB, pn);

//            dv = ls2 + cross(as2, cpi.rB) - ls1 - cross(as1, cpi.rA);

            // tangent impulse
            f32 vt = dot(dv, tangent);
            f32 inc_imp_t = cpi.mass_tangent * (-vt);

            // clamp
            f32 max_friction = f * cc.normal;
            f32 acc_imp_t_old = cc.tangent;
            cc.tangent = clampToRange(cc.tangent  + inc_imp_t, -max_friction, max_friction);
            inc_imp_t = cc.tangent - acc_imp_t_old;

            Vec2 pt = inc_imp_t * tangent;
            ls1 -= imA * (pn + pt);
            as1 -= iiA * (cross(cpi.rA, pn) + cross(cpi.rA, pt));
            ls2 += imB * (pn + pt);
            as2 += iiB * (cross(cpi.rB, pn) + cross(cpi.rB, pt));

            imp_n += inc_imp_n;
            imp_t += inc_imp_t;
        }

        small_io = small_io & (fabsf(imp_n) < maths::EPS && fabsf(imp_t) < maths::EPS);
    }

}

#undef CJ_TPL
#undef CJ_TYPE