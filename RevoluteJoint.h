#pragma once
#include "Joint.h"

namespace Urho3DPhysX
{
    class URHOPX_API RevoluteJoint : public Joint
    {
        URHO3D_OBJECT(RevoluteJoint, Joint);

    public:
        RevoluteJoint(Context* context);
        ~RevoluteJoint();

        static void RegisterObject(Context* context);

        void SetDriveVelocity(float value);
        float GetDriveVelocity() const { return driveVelocity_; }
        void SetDriveForceLimit(float value);
        float GetDriveForceLimit() const { return driveForceLimit_; }
        void SetDriveGearRatio(float value);
        float GetDriveGearRatio() const { return driveGearRatio_; }
        void SetUpperLimit(float value);
        float GetUpperLimit() const { return upperLimit_; }
        void SetLowerLimit(float value);
        float GetLowerLimit() const { return lowerLimit_; }
        void SetContactDistance(float value);
        float GetContactDistance() const { return contactDistance_; }
        void SetLimitEnabled(bool enable);
        bool IsLimitEnabled() const { return limitEnabled_; }
        void SetDriveEnabled(bool enable);
        bool IsDriveEnabled() const { return driveEnabled_; }
        void SetDriveFreeSpin(bool enable);
        bool GetDriveFreeSpin() const { return driveFreeSpin_; }

    protected:
        void CreateJointInternal() override;
        void UpdateLimit();
        float driveVelocity_;
        float driveForceLimit_;
        float driveGearRatio_;
        float lowerLimit_;
        float upperLimit_;
        float contactDistance_;
        bool limitEnabled_;
        bool driveEnabled_;
        bool driveFreeSpin_;
    };
}
