#pragma once
#include "SampleBase.h"
#include <Urho3D/Math/Ray.h>

namespace Urho3DPhysX
{
    class PhysXScene;
}

namespace Urho3D
{
    class DebugRenderer;
}

class RaycastSample : public SampleBase
{
    URHO3D_OBJECT(RaycastSample, SampleBase);

public:
    RaycastSample(Context* context);
    ~RaycastSample();

    void SampleStart() override;
    void Update(float timeStep) override;

private:
    void HandleTriggerEnter(StringHash eventType, VariantMap& eventData);
    Urho3DPhysX::PhysXScene* pxScene_;
    Urho3D::DebugRenderer* dbr_;
    float sweepRadius_;
};
