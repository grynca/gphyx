#ifndef PHYSICSMANAGERT_H
#define PHYSICSMANAGERT_H

#include "gphyx_config.h"
#include "joints.h"
#include "maths/shapes/Shape.h"
#include "maths/shapes/OverlapHelper.h"
#include "types/containers/PolyPool.h"

namespace grynca {

    // keeps internal copy of Speeds for better cache coherence
    // (in hash map with key BodyIdT - Hasher<BodyIdT> must be available)
    template <typename Derived/*CRTP*/, typename BodiesPairHandler>
    class PhysicsManagerT : private SAPManagerC<Derived, gphyx::SAPDomain> {
    public:
        typedef SAPManagerC<Derived, gphyx::SAPDomain> Base;
        typedef PolyPool<JointTypes::getTypesCount(), Joint, Derived> JointsPool;
        typedef ObjFunc<void()> OverlapCbFunc;

        PhysicsManagerT(f32 updates_per_sec = 60);
        Base& accSAP();
        BodiesPairHandler& accBPH();

        void setUPS(f32 updates_per_sec);

        f32 getUpdatesPerSec()const;
        f32 getUpdateDt()const;

        void addCollider(Collider& coll, const Transform& tr, Index body_id);
        void removeCollider(Collider& coll);
        void moveCollider(const Collider& coll, const Vec2& move_vec);
        void updateCollider(const Collider& coll, const Transform& tr);

        // calculates tentative velocity
        void updateVelocity(Body& b, Speed& spd_io);
        void updateContacts();
        void solve();
        void updateTransform(const Speed& spd, Transform& tr_io);
        Vec2 updateTransform2(const Speed& spd, Transform& tr_io);      // returns move vector which caller can use for moving with collider

        u32 getContactsCount()const;
        
        // Joints
        template <typename JointType, typename... InitArgs>
        JointType& addJoint(Index body_id1, Index body_id2, InitArgs&&... args);
        void removeJoint(Index id);

        template <typename JointType>
        const JointType& getJoint(Index id)const;
        template <typename JointType>
        JointType& accJoint(Index id);
        const Joint& getJoint(Index id)const;
        Joint& accJoint(Index id);

        // OverlapCallbacks
        template <typename Functor>
        void setBeginOverlapCb(u32 group1_id, u32 group2_id);
        template <typename Functor>
        void setOnOverlapCb(u32 group1_id, u32 group2_id);
        template <typename Functor>
        void setEndOverlapCb(u32 group1_id, u32 group2_id);

        // setting for multiple groups specified with mask
        template <typename Functor>
        void setBeginOverlapCbs(u32 group1_id, CollisionGroupsMask groups2_mask);
        template <typename Functor>
        void setOnOverlapCbs(u32 group1_id, CollisionGroupsMask groups2_mask);
        template <typename Functor>
        void setEndOverlapCbs(u32 group1_id, CollisionGroupsMask groups2_mask);

        void clear();

        std::string getOverlapCallbacksDebugString()const;

        // in case you want to call it yourself (e.g. do some custom handling + default)
        void defaultPhysicalOverlap();
    protected:
        friend class SAPManagerBase<gphyx::SAPDomain>;

        struct PreStepJoints_ {
            template <typename TP, typename T>
            static void f(Derived& pm);
        };

        struct StepJoints_ {
            template <typename TP, typename T>
            static void f(Derived& pm, bool& small);
        };

        // overriden from base
        bool beforeBoxesOverlap_(gphyx::SAPBox&, gphyx::SAPBox&);
        void afterBoxesOverlap_(gphyx::SAPBox&, gphyx::SAPBox&);

        Derived& getAsD_();
        void initCallbacks_();
        OverlapCbFunc& accCbs_(OverlapCbFunc* cbs_arr, u32 row, u32 col);

        template <typename OnSpeedFunc>
        void solveInner_(const OnSpeedFunc& osf);
        void getSpeedsFromBph_(u32 &spdA_id_out, u32 &spdB_id_out);

        ARect transformBound_(const Collider& coll, const Transform& tr);

        f32 ups_, dt_;
        BodiesPairHandler bph_;
        OverlapHelper ohlp_;
        gphyx::SpeedsHashMapT speeds_;

        // overlap callbacks
        OverlapCbFunc begin_op_cbs_[GPHYX_MAX_CGROUPS*GPHYX_MAX_CGROUPS];
        OverlapCbFunc on_op_cbs_[GPHYX_MAX_CGROUPS*GPHYX_MAX_CGROUPS];
        OverlapCbFunc end_op_cbs_[GPHYX_MAX_CGROUPS*GPHYX_MAX_CGROUPS];

        // contact joints (last only 1 tick)
        fast_vector<ContactJoint<1> > contacts1_;
        fast_vector<ContactJoint<2> > contacts2_;

        // persistant joints
        JointsPool joints_;
    };

    template <typename BodiesPairHandler>
    class PhysicsManagerSimpleT : public PhysicsManagerT<PhysicsManagerSimpleT<BodiesPairHandler>, BodiesPairHandler> {};
}

#undef PMT
#include "PhysicsManagerT.inl"
#endif //PHYSICSMANAGERT_H
