#pragma once
#include "Joint.h"

namespace Urho3DPhysX
{
    class __declspec(dllexport) SphericalJoint : public Joint
    {
        URHO3D_OBJECT(SphericalJoint, Joint);

    public:
        SphericalJoint(Context* context);
        ~SphericalJoint();

        static void RegisterObject(Context* context);

        void SetLimitConeAngles(const Vector2& axesLimit);
        const Vector2& GetLimitConeAngles() const { return limitConeAngles_; }
        void SetContactDistance(float value);
        float GetContactDistance() const { return contactDistance_; }
        void SetLimitEnabled(bool value);
        bool IsLimitEnabled() const { return limitEnabled_; }
    protected:
        void CreateJointInternal() override;
        void UpdateStiffness() override;
        void UpdateDamping() override;
        void ApplyLimitCone();
        Vector2 limitConeAngles_;
        float contactDistance_;
        bool limitEnabled_;
    };
}
