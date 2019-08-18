#pragma once
#include "Joint.h"

namespace Urho3DPhysX
{
    class __declspec(dllexport) DistanceJoint : public Joint
    {
        URHO3D_OBJECT(DistanceJoint, Joint);

    public:
        DistanceJoint(Context* context);
        ~DistanceJoint();

        static void RegisterObject(Context* context);
        ///
        float GetDistance() const;
        void SetMinDistance(float value);
        float GetMinDistance() const { return minDistance_; }
        void SetMaxDistance(float value);
        float GetMaxDistance() const { return maxDistance_; }
        void SetTolerance(float value);
        float GetTolerance() const { return tolerance_; }
        void SetMinDistanceEnabled(bool enable);
        bool IsMinDistanceEnabled() const { return minDistanceEnabled_; }
        void SetMaxDistanceEnabled(bool enable);
        bool IsMaxDistanceEnabled() const { return maxDistanceEnabled_; }
        void SetSpringEnabled(bool enable);
        bool IsSpringEnabled() const { return springEnabled_; }
    protected:
        ///
        void CreateJointInternal() override;
        ///
        void UpdateStiffness() override;;
        ///
        void UpdateDamping() override;
        ///
        float minDistance_;
        float maxDistance_;
        float tolerance_;
        bool minDistanceEnabled_;
        bool maxDistanceEnabled_;
        bool springEnabled_;
    };
}
