#ifndef JOINT_H
#define JOINT_H

namespace grynca {

    class Joint {
    public:
        Index getBodyIdA()const { return bidA_; }
        Index getBodyIdB()const { return bidB_; }

        Index getId()const { return joint_id_; }
    protected:
        template <typename, typename> friend class PhysicsManagerT;

        template <typename BodiesPairHandler>
        void preInit_(Index my_id, BodiesPairHandler& bph) {
            joint_id_ = my_id;
            bidA_ = bph.getBodyIdA();
            bidB_ = bph.getBodyIdB();
        }
        void init_() {}

        Index joint_id_;
        Index bidA_, bidB_;
    };

}

#endif //JOINT_H
