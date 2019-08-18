#pragma once
#include "SampleBase.h"

namespace Urho3DPhysX
{
    class PhysXMaterial;
}
using namespace Urho3DPhysX;

class MaterialsSample : public SampleBase
{
    URHO3D_OBJECT(MaterialsSample, SampleBase);

public:
    MaterialsSample(Context* context);
    ~MaterialsSample();

    void SampleStart() override;
    void Update(float timeStep) override;

    void CreateSlide(const Vector3& position, const String& matName, bool applyToCube = false);
};
