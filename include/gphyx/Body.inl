#include "Body.h"

namespace grynca {

    inline Body::Body()
     : torque_(0.0f), prev_torque_(0.0f)
    {
    }

    void Body::initStatic(const Material& mat) {
        material_ = mat;
        mass_data_.setZero();
    }

    inline void Body::initWithShape(const Material& mat, const Shape& sh) {
        initWithArea(mat, sh.calcInertia(), sh.calcArea());
    }

    inline void Body::initWithArea(const Material& mat, f32 base_inertia, f32 area) {
        f32 mass = mat.calculateMass(area);
        initWithMass(mat, base_inertia, mass);
    }

    inline void Body::initWithMass(const Material& mat, f32 base_inertia, f32 mass) {
        material_ = mat;
        setMassData(base_inertia, mass);
    }

    inline const MassData& Body::getMassData()const {
        return mass_data_;
    }

    inline const Material& Body::getMaterial()const {
        return material_;
    }

    inline Material& Body::accMaterial() {
        return material_;
    }

    inline void Body::setMaterial(const Material& mat, bool recalc_mass) {
        if (recalc_mass) {
            ASSERT(material_.density != 0.0f);
            f32 dscale = mat.density/material_.density;
            mass_data_.mass *= dscale;
            mass_data_.inv_mass = 1.f/mass_data_.mass;
        }
        material_ = mat;
    }

    inline void Body::setMassData(f32 base_inertia, f32 mass) {
        if (mass == 0.0f) {
            mass_data_.setZero();
            return;
        }
        mass_data_.mass = mass;
        mass_data_.inv_mass = 1.f/mass;
        if (base_inertia == 0.0f) {
            mass_data_.inv_inertia = mass_data_.inertia = 0.0f;
        }
        else {
            mass_data_.inertia = mass*base_inertia;
            mass_data_.inv_inertia = 1.f/mass_data_.inertia;
        }
    }

    inline void Body::applyForce(const Vec2& r, const Vec2& force) {
        // this is a little odd, imho force should care how far from center of mass it is applied
        // (but this is taken from Box2D)
        force_ += force;
        torque_ += cross(r, force);
    }

    inline void Body::applyForceAtCenter(const Vec2& force) {
        force_ += force;
    }

    inline void Body::applyTorque(f32 torque) {
        torque_ += torque;
    }

    inline void Body::forceToAcceleration(Vec2& lin_accel_out, f32& ang_accel_out) {
        lin_accel_out = force_*mass_data_.inv_mass;
        ang_accel_out = torque_*mass_data_.inv_inertia;
    }

    inline void Body::clearForces() {
        prev_force_ = force_;
        prev_torque_ = torque_;
        force_.set(0.0f, 0.0f);
        torque_ = 0.0f;
    }

    inline const Vec2& Body::getForce()const {
        return force_;
    }

    inline f32 Body::getTorque()const {
        return torque_;
    }

    inline const Vec2& Body::getPrevForce()const {
        return prev_force_;
    }

    inline f32 Body::getPrevTorque()const {
        return prev_torque_;
    }
}