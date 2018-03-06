#ifndef REVOLUTEJOINT_H
#define REVOLUTEJOINT_H

#include "gphyx/gphyx_domain.h"

namespace grynca {

    class RevoluteJoint : public Joint {
    public:
        RevoluteJoint();

        const Vec2& getAnchorA()const { return anchorA_; }
        const Vec2& getAnchorB()const { return anchorB_; }
        Vec2& accAnchorA() { return anchorA_; acc_impulse_.set(0, 0); }
        Vec2& accAnchorB() { return anchorB_; acc_impulse_.set(0, 0); }
        const f32 getSoftness()const { return softness_; }
        f32& accSoftness() { return softness_; }
        void setSoftness(f32 s) { softness_ = s; }
    protected:
        template <typename, typename> friend class PhysicsManagerT;

        template <typename BodiesPairHandler>
        void preInit_(Index my_id, BodiesPairHandler& bph);
        // anchor point is in global coords
        void init_(const Vec2& anchor_pt);

        template <typename BodiesPairHandler>
        void preStep_(BodiesPairHandler &bph, gphyx::SpeedsHashMapT &speeds, u32 spdA_id, u32 spdB_id, f32 ups);
        void solve_(gphyx::SpeedsHashMapT &speeds, bool &small_io);
    private:
        f32 softness_;
        f32 M_[4];
        Vec2 anchorA_, anchorB_;
        Vec2 rA_, rB_;
        Vec2 bias_, acc_impulse_;
        Body* bA_;
        Body* bB_;
        Transform* trA_;
        Transform* trB_;
        u32 sA_id_, sB_id_;
    };
}

#include "RevoluteJoint.inl"
#endif //REVOLUTEJOINT_H
