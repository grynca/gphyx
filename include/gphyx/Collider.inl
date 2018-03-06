#include "Collider.h"

namespace grynca {


    inline Collider::Collider()
     : collision_group_(0)
    {
        // at default, collides with everything
        collides_with_.set();
    }

    inline const Shape& Collider::getShape()const {
        return shape_;
    }

    inline Shape& Collider::accShape() {
        return shape_;
    }

    inline void Collider::setShape(const Shape& sh) {
        shape_ = sh;
    }

    inline void Collider::calcTightBound() {
        bound_ = shape_.calcARectBound();
    }

    inline void Collider::calcRotationInvariantBound() {
        bound_ = shape_.calcARectBound().calcRotInvBound();
    }

    inline Index Collider::getSapId()const {
        return sap_id_;
    }

    inline Index& Collider::accSapId() {
        return sap_id_;
    }

    inline const ARect& Collider::getBoundARect()const {
        return bound_;
    }

    inline ARect& Collider::accBoundARect() {
        return bound_;
    }

    inline void Collider::setBoundRect(const ARect& b) {
        bound_ = b;
    }

    inline u32 Collider::getCollisionGroup()const {
        return collision_group_;
    }

    inline void Collider::setCollisionGroup(u32 cg) {
        collision_group_ = cg;
    }

    inline const CollisionGroupsMask& Collider::getCollidesWithMask()const {
        return collides_with_;
    }

    inline CollisionGroupsMask& Collider::accCollidesWithMask() {
        return collides_with_;
    }

    inline void Collider::setCollidesWithMask(const CollisionGroupsMask& cw) {
        collides_with_ = cw;
    }

    inline bool Collider::collidesWith(const Collider& b)const {
        return collides_with_[b.getCollisionGroup()]
               && b.getCollidesWithMask()[collision_group_];
    }

}