#pragma once
#include "PhysXUtils.h"
#include <Urho3D/Scene/Component.h>
#include <extensions/PxJoint.h>

using namespace physx;
using namespace Urho3D;

namespace Urho3DPhysX
{

    class RigidActor;

    class URHOPX_API Joint : public Component
    {
        URHO3D_OBJECT(Joint, Component);
    public:
        Joint(Context* context);
        virtual ~Joint();

        static void RegisterObject(Context* context);
        ///
        void ApplyAttributes() override;

        void SetOtherActor(RigidActor* actor);
        ///
        void CreateJoint();
        ///
        void SetTransform(const Vector3& position, const Quaternion& rotation = Quaternion::IDENTITY);
        ///
        void SetPosition(const Vector3& position);
        ///
        const Vector3& GetPosition() const { return position_; }
        ///
        void SetRotation(const Quaternion& rotation);
        ///
        const Quaternion& GetRotation() const { return rotation_; }
        ///
        void SetOtherTransform(const Vector3& position, const Quaternion& rotation = Quaternion::IDENTITY);
        ///
        void SetOtherPosition(const Vector3& position);
        ///
        const Vector3& GetOtherPosition() const { return otherPosition_; }
        ///
        void SetOtherRotation(const Quaternion& rotation);
        ///
        const Quaternion& GetOtherRotation() const { return otherRotation_; }
        ///
        void SetBreakForce(float force);
        ///
        float GetBreakForce() const { return breakForce_; }
        ///
        void SetBreakTorque(float torque);
        ///
        float GetBreakTorque() const { return breakTorque_; }
        ///
        void SetStiffness(float value);
        ///
        float GetStiffness() const { return stiffness_; }
        ///
        void SetDamping(float value);
        ///
        float GetDamping() const { return damping_; }
        ///
        void SetUseSoftLimit(bool use);
        ///
        bool IsUsingSoftLimit() const { return useSoftLimit_; }
        ///
        void SetEnableCollision(bool value);
        ///
        bool GetCollisionEnabled() const { return enableCollision_; }
        ///
        void SetOtherActorNodeIDAttr(unsigned value);
        ///
        unsigned GetOtherActorNodeIDAttr() const { return otherActorNodeID_; }
        ///Release Joint
        void ReleaseJoint(bool removeFromActors);

    protected:
        ///
        virtual void CreateJointInternal() {};
        ///
        virtual void UpdateStiffness() {};
        ///
        virtual void UpdateDamping() {};
        ///
        virtual void UpdateUseSoftLimit() {};
        ///
        //void OnNodeSet(Node* node) override;
        ///
        //void OnMarkedDirty(Node* node) override;
        ///
        Vector3 GetScaledPosition(RigidActor* actor, const Vector3& position);
        ///
        WeakPtr<RigidActor> ownActor_;
        WeakPtr<RigidActor> otherActor_;
        unsigned otherNodeID_;
        PxJoint* joint_;
        //JointType jointType_;
        Vector3 cachedWorldScale_;
        Vector3 position_;
        Quaternion rotation_;
        Vector3 otherPosition_;
        Quaternion otherRotation_;
        float breakForce_;
        float breakTorque_;
        ///
        float stiffness_;
        ///
        float damping_;
        ///
        bool useSoftLimit_;
        ///
        bool enableCollision_;
        ///
        unsigned otherActorNodeID_;
        ///
        bool needCreation_;
    };
}

