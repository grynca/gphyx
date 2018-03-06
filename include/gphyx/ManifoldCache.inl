#include "ManifoldCache.h"

namespace grynca {

    inline void ManifoldCache::invalidate() {
        for (u32 i=0; i<ContactManifold::MAX_SIZE; ++i) {
            cc[i].feature_id = u32(-1);
        }
    }

    inline void ManifoldCache::clear() {
        for (u32 i=0; i<ContactManifold::MAX_SIZE; ++i) {
            cc[i].normal = cc[i].tangent = 0.0f;
        }
    }

    inline void ManifoldCache::update(const ContactManifold& cm) {
        ContactCache cc_copy[ContactManifold::MAX_SIZE];
        for (u32 i=0; i<ContactManifold::MAX_SIZE; ++i) {
            cc_copy[i] = cc[i];
            cc[i].feature_id = u32(-1);
        }

        for (u32 i=0; i<cm.size; ++i) {
            u32 j;
            for (j=0; j<ContactManifold::MAX_SIZE; ++j) {
                if (cm.points[i].feature_id == cc_copy[j].feature_id) {
                    break;
                }
            }
            bool found = (j!=ContactManifold::MAX_SIZE);
            if (found) {
                cc[i].normal = cc_copy[j].normal;
                cc[i].tangent = cc_copy[j].tangent;
            }
            else {
                cc[i].normal = cc[i].tangent = 0.0f;
            }
            cc[i].feature_id = cm.points[i].feature_id;
        }
    }

}