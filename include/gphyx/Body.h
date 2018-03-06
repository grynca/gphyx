#ifndef BODY_H
#define BODY_H

#include "maths/Vec2.h"
#include "maths/shapes/Shape.h"

namespace grynca {

    struct MassData {
        constexpr MassData() : mass(0.0f), inv_mass(0.0f), inertia(0.0f), inv_inertia(0.0f) {}
        void setZero() { mass = inv_mass = inertia = inv_inertia = 0.0f; }

        f32 mass;
        f32 inv_mass;
        f32 inertia;
        f32 inv_inertia;
    };

    struct Material {
        constexpr Material() : density(0.0f), restitution(0.0f), friction(0.0f) {}
        constexpr Material(f32 d, f32 r, f32 f) : density(d), restitution(r), friction(f) {}

        static constexpr Material Rock() {return Material(0.6f, 0.1f, 0.85f);}
        static constexpr Material Wood() {return Material(0.3f, 0.2f, 0.45f);}
        static constexpr Material Metal() {return Material(1.2f, 0.05f, 0.3f);}
        static constexpr Material BouncyBall() {return Material(0.3f, 0.8f, 0.8f);}
        static constexpr Material SuperBall() {return Material(0.3f, 0.95f, 0.9f);}
        static constexpr Material Pillow() {return Material(0.1f, 0.2f, 0.2f);}
        static constexpr Material Static() {return Material(0.0f, 0.4f, 0.5f);}

        f32 calculateMass(f32 area)const { return density*area*gphyx::PX_TO_METER; }

        f32 density;
        f32 restitution;
        f32 friction;
    };

    // l_accel = force/mass
    // a_accel = torque/inertia
    //
    // impulses ... instantenous change of velocity
    //  l_impulse = force*updates_per_sec
    //  a_impulse = torque*updates_per_Sec

    /* e.g. instantly accelerate to target_vel:
     *   auto impulse = (target_vel-curr_vel)*mass*updates_per_sec;
     *   b.applyForceAtCenter(impulse);
     */

    class Body {
    public:
        Body();

        // sets both mass and inertia to 0.0
        void initStatic(const Material& mat);
        void initWithShape(const Material& mat, const Shape& sh);
        void initWithArea(const Material& mat, f32 base_inertia, f32 area);
        // When you dont want to calc mass from material density
        void initWithMass(const Material& mat, f32 base_inertia, f32 mass);

        const MassData& getMassData()const;
        const Material& getMaterial()const;
        Material& accMaterial();

        void setMaterial(const Material& mat, bool recalc_mass = true);
        void setMassData(f32 base_inertia, f32 mass);

        // r is vector from body center of mass to point of application
        //  both r & force should be in frame in which we compute accelerations/velocities (usually global frame)
        void applyForce(const Vec2& r, const Vec2& force);
        void applyForceAtCenter(const Vec2& force);
        void applyTorque(f32 torque);

        void forceToAcceleration(Vec2& lin_accel_out, f32& ang_accel_out);
        void clearForces();

        // get forces scheduled to act this update step
        const Vec2& getForce()const;
        f32 getTorque()const;

        // get forces that were acting on body previous upd. step
        const Vec2& getPrevForce()const;
        f32 getPrevTorque()const;
    private:
        // force is converted to acceleration each tick
        Vec2 force_, prev_force_;
        f32 torque_, prev_torque_;

        Material material_;
        MassData mass_data_;
    };
}

#include "Body.inl"
#endif //BODY_H
