#pragma once

#include "RigidActor.h"
#include <Urho3D/Scene/Node.h>

namespace Urho3DPhysX
{
    class URHOPX_API RigidBody : public RigidActor
    {
        URHO3D_OBJECT(RigidBody, RigidActor);

    public:
        RigidBody(Context* context);
        ~RigidBody();
        ///
        static void RegisterObject(Context* context);
        ///
        bool AttachShape(CollisionShape* shape, bool updateMassAndInteria = true) override;
        ///
        void ApplyAttributes() override;
        ///
        float GetMass() const { return mass_; }
        ///
        void SetMass(float mass);
        ///
        const Vector3& GetCenterOfMass() const { return centerOfMass_; }
        ///
        void SetCenterOfMass(const Vector3& center);
        ///
        bool IsDirty() { return isDirty_; }
        ///
        void MarkDirty() { isDirty_ = true; }
        ///
        void SetLinearDamping(float dumping);
        ///
        float GetLinearDamping() const;
        ///
        void SetAngularDamping(float dumping);
        ///
        float GetAngularDamping() const;
        ///
        void SetMaxLinearVelocity(float value);
        ///
        float GetMaxLinearVelocity() const;

        void SetLinearVelocity(const Vector3& value);
        ///
        Vector3 GetLinearVelocity() const;
        ///
        void SetMaxAngularVelocity(float value);
        ///
        float GetMaxAngularVelocity() const;
        ///
        void SetAngularVelocity(const Vector3& value);
        ///
        Vector3 GetAngularVelocity() const;
        ///
        void UpdateMassAndInertia();
        ///
        void ApplyForce(const Vector3& force, const Vector3& pos, TransformSpace space = TS_LOCAL);
        ///
        void ApplyImpulse(const Vector3& force, const Vector3& pos, TransformSpace space = TS_LOCAL);
        ///
        void ApplyVelocityChange(const Vector3& force, const Vector3& pos, TransformSpace space = TS_LOCAL);
        ///
        void ApplyAcceleration(const Vector3& force, const Vector3& pos, TransformSpace space = TS_LOCAL);
        ///
        void SetCCDEnabled(bool enable);
        ///
        bool IsCCDEnabled() const { return ccdEnabled_; }
        ///
        void SetKinematic(bool kinematic);
        ///
        bool IsKinematic() const { return kinematic_; }

    protected:
        void OnNodeSet(Node* node) override;
        float mass_;
        Vector3 centerOfMass_;
        bool isDirty_;//to be removed
        bool ccdEnabled_;
        bool kinematic_;
    };
}
