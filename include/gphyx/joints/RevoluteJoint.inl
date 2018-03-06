#include "RevoluteJoint.h"
#include "../gphyx_config.h"

namespace grynca {

    inline RevoluteJoint::RevoluteJoint()
     : softness_(0.0f)
    {}

    template <typename BodiesPairHandler>
    inline void RevoluteJoint::preInit_(Index my_id, BodiesPairHandler& bph) {
        Joint::preInit_(my_id, bph);
        trA_ = &bph.accTransformA();
        trB_ = &bph.accTransformB();
    }

    inline void RevoluteJoint::init_(const Vec2& anchor_pt) {
        anchorA_ = (-*trA_).calcMatrix()*anchor_pt;
        anchorB_ = (-*trB_).calcMatrix()*anchor_pt;
    }

    template <typename BodiesPairHandler>
    inline void RevoluteJoint::preStep_(BodiesPairHandler &bph, gphyx::SpeedsHashMapT &speeds, u32 spdA_id, u32 spdB_id, f32 ups) {
        bA_ = &bph.accBodyA();
        bB_ = &bph.accBodyB();
        trA_ = &bph.accTransformA();
        trB_ = &bph.accTransformB();
        sA_id_ = spdA_id;
        sB_id_ = spdB_id;

        // Pre-compute anchors, mass matrix, and bias.
        rA_ = anchorA_.rotate(trA_->getRotDir());
        rB_ = anchorB_.rotate(trB_->getRotDir());

        f32 imA = bA_->getMassData().inv_mass;
        f32 imB = bB_->getMassData().inv_mass;
        f32 iiA = bA_->getMassData().inv_inertia;
        f32 iiB = bB_->getMassData().inv_inertia;

        f32 im_sum = imA+imB;
        f32 v1 = im_sum + iiA*rA_.getY()*rA_.getY() + iiB*rB_.getY()*rB_.getY() + softness_;
        f32 v2 = -iiA * rA_.getX() * rA_.getY() - iiB*rB_.getX()*rB_.getY();
        f32 v3 = im_sum + iiA*rA_.getX()*rA_.getX() + iiB*rB_.getX()*rB_.getX() + softness_;

        // [ v1 v2 ]   & invert
        // [ v2 v3 ]

        f32 det = v1*v3 - v2*v2;
        ASSERT(det != 0.0f);
        det = 1.0f/det;
        M_[0] = det*v3;
        M_[1] = M_[2] = -det*v2;
        M_[3] = det*v1;

        Vec2 p1 = trA_->getPosition() + rA_;
        Vec2 p2 = trB_->getPosition() + rB_;
        Vec2 dp = p2 - p1;

        bias_ = -gphyx::JOINTS_BETA*ups*dp;

        Speed& sA = speeds.accItem(sA_id_);
        Speed& sB = speeds.accItem(sB_id_);
        Vec2& ls1 = sA.accLinearSpeed();
        Angle& as1 = sA.accAngularSpeed();
        Vec2& ls2 = sB.accLinearSpeed();
        Angle& as2 = sB.accAngularSpeed();

        // Apply accumulated impulse (needed for warm starting)
        ls1 -= imA * acc_impulse_;
        as1 -= iiA * cross(rA_, acc_impulse_);
        ls2 += imB * acc_impulse_;
        as2 += iiB * cross(rB_, acc_impulse_);
    }

    inline void RevoluteJoint::solve_(gphyx::SpeedsHashMapT &speeds, bool &small_io) {
        Speed& sA = speeds.accItem(sA_id_);
        Speed& sB = speeds.accItem(sB_id_);
        Vec2& ls1 = sA.accLinearSpeed();
        Angle& as1 = sA.accAngularSpeed();
        Vec2& ls2 = sB.accLinearSpeed();
        Angle& as2 = sB.accAngularSpeed();

        f32 imA = bA_->getMassData().inv_mass;
        f32 imB = bB_->getMassData().inv_mass;
        f32 iiA = bA_->getMassData().inv_inertia;
        f32 iiB = bB_->getMassData().inv_inertia;

        Vec2 dv = ls2 + cross(as2, rB_) - ls1 - cross(as1, rA_);
        dv = bias_ - dv - softness_*acc_impulse_;

        Vec2 impulse(M_[0]*dv.getX()+M_[2]*dv.getX(), M_[1]*dv.getX()+M_[3]*dv.getY());

        ls1 -= imA * impulse;
        as1 -= iiA * cross(rA_, impulse);
        ls2 += imB * impulse;
        as2 += iiB * cross(rB_, impulse);
        acc_impulse_ += impulse;

        small_io = small_io & (fabsf(impulse.getX()) < maths::EPS && fabsf(impulse.getY()) < maths::EPS);
    }

}