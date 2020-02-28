#pragma once
#include "Joint.h"

namespace Urho3DPhysX
{
    class URHOPX_API FixedJoint : public Joint
    {
        URHO3D_OBJECT(FixedJoint, Joint);

    public:
        FixedJoint(Context* context);
        ~FixedJoint();

        static void RegisterObject(Context* context);

        void SetProjectionLinearTolerance(float value);
        float GetProjectionLinearTolerance() const { return projectionLinearTolerance_; }
        void SetProjectionAngularTolerance(float value);
        float GetProjectionAngularTolerance() const { return projectionAngularTolerance_; }

    protected:
        void CreateJointInternal() override;
        float projectionLinearTolerance_;
        float projectionAngularTolerance_;
    };
}
