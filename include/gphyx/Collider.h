#ifndef COLLIDER_H
#define COLLIDER_H

#include "maths/shapes/ARect.h"
#include "gphyx_config.h"

namespace grynca {

    typedef Mask<GPHYX_MAX_CGROUPS> CollisionGroupsMask;

    class Collider {
    public:
        Collider();

        const Shape& getShape()const;
        Shape& accShape();
        void setShape(const Shape& sh);

        // tighter bound (is not rotation invariant)
        //  e.g. for: static bodies, circle
        void calcTightBound();

        // calculate larger rotation invariant bound
        //      e.g. needed for dynamic non-circle shapes
        void calcRotationInvariantBound();

        Index getSapId()const;
        Index& accSapId();

        const ARect& getBoundARect()const;
        ARect& accBoundARect();
        void setBoundRect(const ARect& b);

        u32 getCollisionGroup()const;
        void setCollisionGroup(u32 cg);

        const CollisionGroupsMask& getCollidesWithMask()const;
        CollisionGroupsMask& accCollidesWithMask();
        void setCollidesWithMask(const CollisionGroupsMask& cw);

        bool collidesWith(const Collider& b)const;
    private:
        Shape shape_;
        ARect bound_;
        Index sap_id_;

        CollisionGroupsMask collides_with_;
        u32 collision_group_;
    };

}

#include "Collider.inl"
#endif //COLLIDER_H
