#pragma once
#include "RigidBody.h"

namespace Urho3DPhysX
{
    class URHOPX_API DynamicBody : public RigidBody
    {
        URHO3D_OBJECT(DynamicBody, RigidBody);

    public:
        using RigidBody::ApplyImpulse;
        using RigidBody::ApplyForce;
        using RigidBody::ApplyVelocityChange;
        using RigidBody::ApplyAcceleration;

        DynamicBody(Context* context);
        ~DynamicBody();

        static void RegisterObject(Context* context);
        ///
        void OnSetEnabled() override;
        ///apply force, will autoawake body
        void ApplyForce(const Vector3& force);
        ///apply impulse, will awake body
        void ApplyImpulse(const Vector3& force);
        ///change velocity
        void ApplyVelocityChange(const Vector3& force);
        ///apply acceleration
        void ApplyAcceleration(const Vector3& force);
        ///apply torque
        void ApplyTorque(const Vector3& torque);
        ///apply torque impulse
        void ApplyTorqueImpulse(const Vector3& torque);
        ///
        void ApplyTorqueVelocityChange(const Vector3& torque);
        ///
        void ApplyTorqueAcceleration(const Vector3& torque);
        ///
        void SetSleepThreshold(float value);
        ///
        float GetSleepThreshold() const;
        ///
        bool IsSleeping() const;
        ///
        void WakeUp();
        ///
        void PutToSleep();
        ///
        void SetStabilizationThreshold(float value);
        ///
        float GetStabilizationThreshold() const;
        ///
        void SetContactReportTheshold(float value);
        ///
        float GetContactReportThreshold() const;
        ///
        void SetLockLinearX(bool lock);
        ///
        bool GetLockLinearX() const;
        ///
        void SetLockLinearY(bool lock);
        ///
        bool GetLockLinearY() const;
        ///
        void SetLockLinearZ(bool lock);
        ///
        bool GetLockLinearZ() const;
        ///
        void SetLockLinear(const IntVector3& lock);
        ///
        IntVector3 GetLockLinear() const;
        ///
        void SetLockAngularX(bool lock);
        ///
        bool GetLockAngularX() const;
        ///
        void SetLockAngularY(bool lock);
        ///
        bool GetLockAngularY() const;
        ///
        void SetLockAngularZ(bool lock);
        ///
        bool GetLockAngularZ() const;
        ///
        void SetLockAngular(const IntVector3& lock);
        ///
        IntVector3 GetLockAngular() const;
        ///
        void SetKinematicTarget(const Vector3& position, const Quaternion& rotation);
        ///
        bool GetKinematicTarget(Vector3& position, Quaternion& rotation);
    protected:
        ///
        void OnMarkedDirty(Node* node) override;
        ///
        void OnJointAdded(Joint* joint) override;
        ///
        void OnJointRemoved(Joint* joint) override;
    };
}
