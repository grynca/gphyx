#ifndef GPHYX_CONFIG_H
#define GPHYX_CONFIG_H

#ifndef GPHYX_MAX_CGROUPS
#   define GPHYX_MAX_CGROUPS 64
#endif

namespace grynca {
    namespace gphyx {
        static const f32 CONTACTS_BETA = 0.15f;
        static const f32 JOINTS_BETA = 0.2f;
        static const f32 PENETRATION_SLOP = 0.03f;
        //static const f32 RESTITUTION_SLOP = 0.05f;
        static const f32 CONTACT_MAX_ITERS = 10;
        static const f32 PX_TO_METER = 0.01f;
    }
}

#endif //GPHYX_CONFIG_H
