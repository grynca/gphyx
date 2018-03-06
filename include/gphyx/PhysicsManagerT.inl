#include "PhysicsManagerT.h"
#include "gphyx_config.h"
#include <sstream>

#define PM_TPL template <typename D, typename BPH>
#define PM_TYPE PhysicsManagerT<D, BPH>

namespace grynca {

    PM_TPL
    inline PM_TYPE::PhysicsManagerT(f32 updates_per_sec)
     : ups_(updates_per_sec), dt_(1.0f/ups_)
    {
        joints_.template initTypes<JointTypes>();
        initCallbacks_();
    }

    PM_TPL
    inline typename PM_TYPE::Base& PM_TYPE::accSAP() {
        return *(Base*)this;
    }

    PM_TPL
    inline BPH& PM_TYPE::accBPH() {
        return bph_;
    }

    PM_TPL
    inline void PM_TYPE::setUPS(f32 updates_per_sec) {
        ups_ = updates_per_sec;
        dt_ = 1.0f/ups_;
    }

    PM_TPL
    inline f32 PM_TYPE::getUpdatesPerSec()const {
        return ups_;
    }

    PM_TPL
    inline f32 PM_TYPE::getUpdateDt()const {
        return dt_;
    }

    PM_TPL
    void PM_TYPE::addCollider(Collider& coll, const Transform& tr, Index body_id) {
        ARect bound_gl = transformBound_(coll, tr);
        Base::addBox(coll.accSapId(), (f32*)bound_gl.accBounds(), body_id);
    }

    PM_TPL
    void PM_TYPE::removeCollider(Collider& coll) {
        Base::removeBox(coll.getSapId());
        coll.accSapId().makeInvalid();
    }

    PM_TPL
    inline void PM_TYPE::moveCollider(const Collider& coll, const Vec2& move_vec) {
        Base::moveBox(coll.getSapId(), (f32*)move_vec.getDataPtr());
    }

    PM_TPL
    inline void PM_TYPE::updateCollider(const Collider& coll, const Transform& tr) {
        ARect bound_gl = transformBound_(coll, tr);
        Base::updateBox(coll.getSapId(), (f32*)bound_gl.accBounds());
    }

//    PM_TPL
//    inline void PM_TYPE::initBody(Body& body, Collider& bound, Index body_id, const Material& mat, const Transform& tr, Shape& shape) {
//        initBody(body, bound, body_id, mat, tr, shape, mat.calculateMass(shape.calcArea()));
//    }
//
//    PM_TPL
//    inline void PM_TYPE::initBody(Body& body, Collider& bound, Index body_id, const Material& mat, const Transform& tr, const Shape& shape, f32 mass) {
//        body.initWithMass(mat, shape.calcInertia(), mass);
//        ARect& bound_rect = bound.accBoundARect();
//        bound_rect = shape.calcARectBound().calcRotInvBound();
//        ARect bound_gl = bound_rect;
//        bound_gl.accBounds()[0] += tr.getPosition();
//        bound_gl.accBounds()[1] += tr.getPosition();
//        Base::addBox(bound.accSapId(), (f32*)bound_gl.accBounds(), body_id);
//    }
//
//    PM_TPL
//    inline void PM_TYPE::initStaticBody(Body& body, Collider& bound, Index body_id, const Material& mat, const Transform& tr, const Shape& shape) {
//        body.initWithMass(mat, shape.calcInertia(), 0.0f);
//        ARect& bound_rect = bound.accBoundARect();
//        bound_rect = shape.calcARectBound();
//
//        ARect bound_gl = bound_rect;
//        bound_gl.transform(tr);
//        Base::addBox(bound.accSapId(), (f32*)bound_gl.accBounds(), body_id);
//    }

    PM_TPL
    inline void PM_TYPE::updateVelocity(Body& b, Speed& spd_io) {
        Vec2 lin_acc;
        f32 ang_acc;
        b.forceToAcceleration(lin_acc, ang_acc);
        spd_io.accLinearSpeed() += lin_acc*dt_;
        spd_io.accAngularSpeed() += ang_acc*dt_;
        b.clearForces();
    }

    PM_TPL
    inline void PM_TYPE::updateContacts() {
        PROFILE_BLOCK("PhysicsManager::SolveContacts()");
        u32 overlaps_cnt = Base::getOverlapsCount();
        for (u32 i=0; i<overlaps_cnt; ++i) {
            Index b1_id, b2_id;

            gphyx::SAPOverlapData* sod;
            Base::getOverlapWithData(i, b1_id, b2_id, sod);
            bph_.cache_ = &sod->cache;
            Index bidA = Base::getBox(b1_id).getClientData();
            Index bidB = Base::getBox(b2_id).getClientData();
            bph_.setBodyA(bidA);
            bph_.setBodyB(bidB);

            Transform& tr1 = bph_.accTransformA();
            Transform& tr2 = bph_.accTransformB();
            Collider& coll1 = bph_.accColliderA();
            Collider& coll2 = bph_.accColliderB();
            ohlp_.set(coll1.getShape(), coll2.getShape(), tr1, tr2);

            // check if inner shapes overlap
            if (ohlp_.overlaps()) {
                u32 groupA = coll1.getCollisionGroup();
                u32 groupB = coll2.getCollisionGroup();
                if (!sod->shape_overlap) {
                    sod->shape_overlap = true;
                    // call begin overlap callback
                    accCbs_(begin_op_cbs_, groupA, groupB)();
                }
                // call on overlap callback
                accCbs_(on_op_cbs_, groupA, groupB)();
            }
            else if (sod->shape_overlap) {
                sod->shape_overlap = false;
                u32 groupA = coll1.getCollisionGroup();
                u32 groupB = coll2.getCollisionGroup();
                // call end overlap callback
                accCbs_(end_op_cbs_, groupA, groupB)();
                bph_.cache_->invalidate();
            }
        }
    }

    PM_TPL
    inline void PM_TYPE::solve() {
        solveInner_([this](Index bid, Speed& spd) {
            // store speeds back to bodies
            bph_.setBodyA(bid);
            bph_.accSpeedA() = spd;
        });
    }

    PM_TPL
    inline void PM_TYPE::updateTransform(const Speed& spd, Transform& tr_io) {
        tr_io.move(spd.getLinearSpeed()*dt_);
        tr_io.rotate(spd.getAngularSpeed()*dt_);
    }

    PM_TPL
    inline Vec2 PM_TYPE::updateTransform2(const Speed& spd, Transform& tr_io) {
        Vec2 move_vec = spd.getLinearSpeed()*dt_;
        // update pos
        tr_io.move(move_vec);
        tr_io.rotate(spd.getAngularSpeed()*dt_);
        return move_vec;
    }

    PM_TPL
    inline u32 PM_TYPE::getContactsCount()const {
        return u32(contacts1_.size() + contacts2_.size());
    }

    PM_TPL
    template <typename JointType, typename... InitArgs>
    inline JointType& PM_TYPE::addJoint(Index id1, Index id2, InitArgs&&... args) {
        bph_.setBodyA(id1);
        bph_.setBodyB(id2);
        Index jid;
        JointType& joint = joints_.template add<JointType>(jid);
        joint.preInit_(jid, bph_);
        joint.init_(std::forward<InitArgs>(args)...);
        return joint;
    }

    PM_TPL
    inline void PM_TYPE::removeJoint(Index id) {
        joints_.removeItem(id);
    }

    PM_TPL
    template <typename JointType>
    inline const JointType& PM_TYPE::getJoint(Index id)const {
        return joints_.template getItem<JointType>(id);
    }

    PM_TPL
    template <typename JointType>
    inline JointType& PM_TYPE::accJoint(Index id) {
        return joints_.template accItem<JointType>(id);
    }

    PM_TPL
    inline const Joint& PM_TYPE::getJoint(Index id)const {
        return joints_.getItem(id);
    }

    PM_TPL
    inline Joint& PM_TYPE::accJoint(Index id) {
        return joints_.accItem(id);
    }

    PM_TPL
    template <typename Functor>
    inline void PM_TYPE::setBeginOverlapCb(u32 group1_id, u32 group2_id) {
        if (group1_id > group2_id) {
            std::swap(group1_id, group2_id);
        }
        accCbs_(begin_op_cbs_, group1_id, group2_id).template bind<Functor>(&getAsD_());
    }

    PM_TPL
    template <typename Functor>
    inline void PM_TYPE::setOnOverlapCb(u32 group1_id, u32 group2_id) {
        if (group1_id > group2_id) {
            std::swap(group1_id, group2_id);
        }
        accCbs_(on_op_cbs_, group1_id, group2_id).template bind<Functor>(&getAsD_());
    }

    PM_TPL
    template <typename Functor>
    inline void PM_TYPE::setEndOverlapCb(u32 group1_id, u32 group2_id) {
        if (group1_id > group2_id) {
            std::swap(group1_id, group2_id);
        }
        accCbs_(end_op_cbs_, group1_id, group2_id).template bind<Functor>(&getAsD_());
    }

    PM_TPL
    template <typename Functor>
    inline void PM_TYPE::setBeginOverlapCbs(u32 group1_id, CollisionGroupsMask groups2_mask) {
        LOOP_SET_BITS(groups2_mask, it) {
            setBeginOverlapCb<Functor>(group1_id, it.getPos());
        }
    }

    PM_TPL
    template <typename Functor>
    inline void PM_TYPE::setOnOverlapCbs(u32 group1_id, CollisionGroupsMask groups2_mask) {
        LOOP_SET_BITS(groups2_mask, it) {
            setOnOverlapCb<Functor>(group1_id, it.getPos());
        }
    }

    PM_TPL
    template <typename Functor>
    inline void PM_TYPE::setEndOverlapCbs(u32 group1_id, CollisionGroupsMask groups2_mask) {
        LOOP_SET_BITS(groups2_mask, it) {
            setEndOverlapCb<Functor>(group1_id, it.getPos());
        }
    }

    PM_TPL
    inline void PM_TYPE::clear() {
        Base::clear();
        contacts1_.clear();
        contacts2_.clear();
        joints_.clear();
    }

    PM_TPL
    inline std::string PM_TYPE::getOverlapCallbacksDebugString()const {
        std::stringstream ss;
//        ss << "Collision callbacks: (P-physical, C-custom, N-no collision)" << std::endl;
//        ss << "  ";
//        for (u32 i=0; i<GPHYX_MAX_CGROUPS; ++i) {
//            ss << i << " ";
//        }
//        ss << std::endl;
//        for (u32 g1=0; g1<GPHYX_MAX_CGROUPS; ++g1) {
//            ss << g1 << " ";
//            for (u32 g2=0; g2<GPHYX_MAX_CGROUPS; ++g2) {
//                auto& cbs = getOverlapsCbs(g1, g2);
//                if (cbs == no_collision_cbs_) {
//                    ss << "N ";
//                }
//                else if (cbs == def_physical_cbs_) {
//                    ss << "P ";
//                }
//                else {
//                    ss << "C ";
//                }
//            }
//            ss << std::endl;
//        }
        return ss.str();
    }

    PM_TPL
    template <typename TP, typename T>
    void PM_TYPE::PreStepJoints_::f(D& pm) {
        // static
        pm.joints_.template forEach<T>([&pm](T& item) {
            pm.bph_.setBodyA(item.getBodyIdA());
            pm.bph_.setBodyB(item.getBodyIdB());
            u32 spdA_id, spdB_id;
            pm.getSpeedsFromBph_(spdA_id, spdB_id);
            item.preStep_(pm.bph_, pm.speeds_, spdA_id, spdB_id, pm.ups_);
        });
    }

    PM_TPL
    template <typename TP, typename T>
    void PM_TYPE::StepJoints_::f(D& pm, bool& small) {
        // static
        pm.joints_.template forEach<T>([&pm, &small](T& item) {
            item.solve_(pm.speeds_, small);
        });
    }

    PM_TPL
    inline bool PM_TYPE::beforeBoxesOverlap_(gphyx::SAPBox& b1, gphyx::SAPBox& b2) {
        Index bidA = b1.getClientData();
        Index bidB = b2.getClientData();

        bph_.setBodyA(bidA);
        bph_.setBodyB(bidB);
        Collider& collA = bph_.accColliderA();
        Collider& collB = bph_.accColliderB();
        // filter by collision groups
        return collA.collidesWith(collB);
    }

    PM_TPL
    inline void PM_TYPE::afterBoxesOverlap_(gphyx::SAPBox& b1, gphyx::SAPBox& b2) {
        gphyx::SAPOverlapData* sod = Base::findOverlap(b1, b2);
        if (sod->shape_overlap) {
            sod->shape_overlap = false;
            Index bidA = b1.getClientData();
            Index bidB = b2.getClientData();
            bph_.setBodyA(bidA);
            bph_.setBodyB(bidB);
            Collider& collA = bph_.accColliderA();
            Collider& collB = bph_.accColliderB();
            u32 groupA = collA.getCollisionGroup();
            u32 groupB = collB.getCollisionGroup();
            // call end overlap callback
            accCbs_(end_op_cbs_, groupA, groupB)();
        }
    }

    PM_TPL
    inline D& PM_TYPE::getAsD_() {
        return *(D*)this;
    }

    PM_TPL
    inline void PM_TYPE::initCallbacks_() {

        SIMPLE_FUNCTOR(DefaultPhysicalOn, (D* pm) {
            pm->defaultPhysicalOverlap();
        });

        SIMPLE_FUNCTOR(SwapEntsCallbackBefore, (D* pm) {
            Collider& collA = pm->accBPH().accColliderA();
            Collider& collB = pm->accBPH().accColliderB();
            u32 groupA = collA.getCollisionGroup();
            u32 groupB = collB.getCollisionGroup();
            ASSERT(groupA > groupB);
            pm->accBPH().swapOrder();
            return pm->accCbs_(pm->begin_op_cbs_, groupB, groupA)();
            pm->accBPH().swapOrder();
        });

        SIMPLE_FUNCTOR(SwapEntsCallbackOn, (D* pm) {
            Collider& collA = pm->accBPH().accColliderA();
            Collider& collB = pm->accBPH().accColliderB();
            u32 groupA = collA.getCollisionGroup();
            u32 groupB = collB.getCollisionGroup();
            ASSERT(groupA > groupB);
            pm->accBPH().swapOrder();
            pm->accCbs_(pm->on_op_cbs_, groupB, groupA)();
            pm->accBPH().swapOrder();
        });

        SIMPLE_FUNCTOR(SwapEntsCallbackAfter, (D* pm) {
            Collider& collA = pm->accBPH().accColliderA();
            Collider& collB = pm->accBPH().accColliderB();
            u32 groupA = collA.getCollisionGroup();
            u32 groupB = collB.getCollisionGroup();
            ASSERT(groupA > groupB);
            pm->accBPH().swapOrder();
            return pm->accCbs_(pm->end_op_cbs_, groupB, groupA)();
            pm->accBPH().swapOrder();
        });

        // default overlap
        for (u32 i=0; i<ARRAY_SIZE(begin_op_cbs_); ++i) {
            begin_op_cbs_[i].template bind<EmptyFunctor>();
            on_op_cbs_[i].template bind<DefaultPhysicalOn>(&getAsD_());
            end_op_cbs_[i].template bind<EmptyFunctor>();
        }

        // register swap callbacks
        for (u32 i=1; i<GPHYX_MAX_CGROUPS; ++i) {
            for (u32 j=0; j<i; ++j) {
                accCbs_(begin_op_cbs_, i, j).template bind<SwapEntsCallbackBefore>(&getAsD_());
                accCbs_(on_op_cbs_, i, j).template bind<SwapEntsCallbackOn>(&getAsD_());
                accCbs_(end_op_cbs_, i, j).template bind<SwapEntsCallbackAfter>(&getAsD_());
            }
        }
    }

    PM_TPL
    inline typename PM_TYPE::OverlapCbFunc& PM_TYPE::accCbs_(OverlapCbFunc* cbs_arr, u32 row, u32 col) {
        return cbs_arr[row*GPHYX_MAX_CGROUPS+col];
    }

    PM_TPL
    inline void PM_TYPE::defaultPhysicalOverlap() {
        ohlp_.calcContactG();
        const ContactManifold& cm = ohlp_.getContactManifoldG();

        u32 spdA_id, spdB_id;
        getSpeedsFromBph_(spdA_id, spdB_id);

        switch (cm.size) {
            case 1:
                contacts1_.emplace_back();
                contacts1_.back().init(ups_, bph_, speeds_, spdA_id, spdB_id, cm, bph_.cache_);
                break;
            case 2:
                contacts2_.emplace_back();
                contacts2_.back().init(ups_, bph_, speeds_, spdA_id, spdB_id, cm, bph_.cache_);
                break;
        }
    }

    PM_TPL
    template <typename OnSpeedFunc>
    inline void PM_TYPE::solveInner_(const OnSpeedFunc& osf) {
        PROFILE_BLOCK("PhysicsManager::solve()");

        // pre-step joints
        JointTypes::callOnTypes<PreStepJoints_>(getAsD_());

        for (u32 iter=0; iter<gphyx::CONTACT_MAX_ITERS; ++iter) {
            bool small = true;
            for (u32 cid=0; cid<contacts1_.size(); ++cid) {
                contacts1_[cid].solve(speeds_, small);
            }
            for (u32 cid=0; cid<contacts2_.size(); ++cid) {
                contacts2_[cid].solve(speeds_, small);
            }

            JointTypes::callOnTypes<StepJoints_>(getAsD_(), small);

            if (small) {
                break;
            }
        }

        // save updated speeds back to bodies
        for (u32 i=0; i<speeds_.getItemsCount(); ++i) {
            Index bid = speeds_.getKey(i);
            Speed& spd = speeds_.accItem(i);
            osf(bid, spd);
        }

        contacts1_.clear();
        contacts2_.clear();
        speeds_.clear();
    }

    PM_TPL
    inline void PM_TYPE::getSpeedsFromBph_(u32 &spdA_id_out, u32 &spdB_id_out) {
        // Add speeds to hash-map if not already contained
        bool added = false;
        Speed* spdA = speeds_.findOrAddItem(bph_.getBodyIdA(), added);
        if (added) {
            new (spdA) Speed(bph_.accSpeedA());
        }
        spdA_id_out = speeds_.getItemIndex(spdA);

        added = false;
        Speed* spdB = speeds_.findOrAddItem(bph_.getBodyIdB(), added);
        if (added) {
            new (spdB) Speed(bph_.accSpeedB());
        }
        spdB_id_out = speeds_.getItemIndex(spdB);
    }

    PM_TPL
    inline ARect PM_TYPE::transformBound_(const Collider& coll, const Transform& tr) {
        // presume that rect is rotatio invariant,
        //  so dont change size due to rotation
        Vec2 half_size = coll.getBoundARect().getSize()*tr.getScale()*0.5f;
        Vec2 center = coll.getBoundARect().getLeftTop() + half_size;
        center = center.rotate(tr.getRotDir()) + tr.getPosition();
        // rotate/scale could flip vertices, make sure lefttop is first
        half_size.setX(fabsf(half_size.getX()));
        half_size.setY(fabsf(half_size.getY()));
        return ARect(center-half_size, half_size*2);
    }
}

#undef PM_TPL
#undef PM_TYPE