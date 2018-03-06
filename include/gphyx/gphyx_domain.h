#ifndef GPHYX_DOMAIN_TYPES_H
#define GPHYX_DOMAIN_TYPES_H

#include "SAP.h"
#include "ManifoldCache.h"
#include "types/containers/HashMap.h"

namespace grynca {

    // fw
    class Speed;

    namespace gphyx {

        struct SAPOverlapData {
            SAPOverlapData() : shape_overlap(false) {}

            bool shape_overlap;
            ManifoldCache cache;
        };

        typedef HashMap<Speed, Index, Hasher<Index> > SpeedsHashMapT;
        typedef SAPDomain2D<Index, SAPOverlapData> SAPDomain;
        typedef SAP::SAPBox<SAPDomain> SAPBox;
    }
}

#endif //GPHYX_DOMAIN_TYPES_H
