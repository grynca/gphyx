#ifndef MANIFOLDCACHE_H
#define MANIFOLDCACHE_H

namespace grynca {

    struct ContactCache {
        u32 feature_id;

        // stored accumulated impulses
        f32 normal;
        f32 tangent;
    };

    struct ManifoldCache {
        void invalidate();
        void clear();

        void update(const ContactManifold& cm);

        ContactCache cc[ContactManifold::MAX_SIZE];
    };

}

#include "ManifoldCache.inl"
#endif //MANIFOLDCACHE_H
