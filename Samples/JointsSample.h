#pragma once
#include "SampleBase.h"

namespace Urho3D
{
    class DebugRenderer;
}
namespace Urho3DPhysX
{
    class PhysXScene;
}

using namespace Urho3DPhysX;

class JointsSample : public SampleBase
{
    URHO3D_OBJECT(JointsSample, SampleBase);

public:
    JointsSample(Context* context);
    ~JointsSample();

    void SampleStart() override;
    void Update(float timeStep) override;

private:
    void CreateSphericalJoints();
    void CreateFixedJoints();
    void CreateDistanceJoints();
    void CreatePrismaticJoints();
    void CreateRevoluteJoints();
    void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData);
    PhysXScene* pxScene_;
    DebugRenderer* dbr_;
};
