#ifndef GPHYX_FUNCS_H
#define GPHYX_FUNCS_H

#include "maths/Vec2.h"

namespace grynca {
    namespace gphyx {

        inline Vec2 calcRelativeVelocityAt(const Vec2& ls1, const Vec2& ls2, f32 as1, f32 as2, const Vec2& r1, const Vec2& r2) {
            return ls2  + cross(as1, r2) - ls1 - cross(as2, r1);
        }

        inline f32 calcRotationTorque(f32 target_rot, f32 curr_rot, f32 curr_avel, f32 inertia, f32 ups, f32 udt /* = 1.0f/ups */, u32 lookahead = 20) {
            f32 next_rot = curr_rot + curr_avel * udt * lookahead;
            f32 rot_delta = Angle(target_rot - next_rot).normalize();
            return rot_delta * (inertia* ups);
        }

        inline Vec2 calcMovementForce(const Vec2& target_pos, const Vec2& curr_pos, const Vec2& curr_lvel, f32 mass, f32 ups, f32 udt /*= 1.0f/ups */, u32 lookahead = 20) {
            Vec2 next_pos = curr_pos + curr_lvel * udt * lookahead;
            Vec2 pos_delta = target_pos - next_pos;
            return pos_delta * (mass * ups);
        }

    }
}

#endif //GPHYX_FUNCS_H
