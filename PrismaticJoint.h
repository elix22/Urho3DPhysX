#pragma once
#include "Joint.h"

namespace Urho3DPhysX
{
    class URHOPX_API PrismaticJoint : public Joint
    {
        URHO3D_OBJECT(PrismaticJoint, Joint);

    public:
        PrismaticJoint(Context* context);
        ~PrismaticJoint();

        static void RegisterObject(Context* context);

        void SetUpperLimit(float value);
        float GetUpperLimit() const { return upperLimit_; }
        void SetLowerLimit(float value);
        float GetLowerLimit() const { return lowerLimit_; }
        void SetContactDistance(float value);
        float GetContactDistance() const { return contactDistance_; }

        void SetLimitEnabled(bool enable);
        bool IsLimitEnabled() const { return limitEnabled_; }

    protected:
        void CreateJointInternal() override;
        void UpdateLimit();
        void UpdateStiffness() override;
        void UpdateDamping() override;

        float upperLimit_;
        float lowerLimit_;
        float contactDistance_;
        bool limitEnabled_;
    };
}
