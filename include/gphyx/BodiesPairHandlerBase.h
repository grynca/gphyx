#ifndef BODIESPAIRHANDLERBASE_H
#define BODIESPAIRHANDLERBASE_H

#include "gphyx_domain.h"

namespace grynca {

    // fw
    class ManifoldCache;

    class BodiesPairHandlerBase {
    public:
        Index getBodyIdA()const { return bidA_; }
        Index getBodyIdB()const { return bidB_; }

        // override if needed
        void setBodyA(Index bid) { bidA_ = bid; }
        void setBodyB(Index bid) { bidB_ = bid; }
        void swapOrder() { std::swap(bidA_, bidB_); }

// functions bellow must be implemented by derived class
//
//            Body& accBodyA();
//            Speed& accSpeedA();
//            Transform& accTransformA();
//            Bound& accColliderA();
//
//            Body& accBodyB();
//            Speed& accSpeedB();
//            Transform accTransformB();
//            Bound& accColliderB();
    protected:
        template <typename, typename> friend class PhysicsManagerT;

        ManifoldCache* cache_;
        Index bidA_, bidB_;
    };
}

#endif //BODIESPAIRHANDLERBASE_H
