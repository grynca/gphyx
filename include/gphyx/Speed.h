#ifndef SPEED_H
#define SPEED_H

#include "maths/Vec2.h"

namespace grynca {

    class Speed {
    public:
        Speed() {}
        Speed(const Vec2& ls, Angle as) : ls_(ls), as_(as) {}

        Angle getAngularSpeed()const { return as_; }
        const Vec2& getLinearSpeed()const { return ls_; }
        Angle& accAngularSpeed() { return as_; }
        Vec2& accLinearSpeed() { return ls_; }
        void setAngularSpeed(const Angle& as) { as_ = as; }
        void setLinearSpeed(const Vec2& ls) { ls_ = ls; }

        void makeZero() { ls_.set(0.0f, 0.0f); as_=0.0f; }
    private:
        Vec2 ls_;
        Angle as_;
    };
}

#endif //SPEED_H
