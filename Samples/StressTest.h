#pragma once
#include "SampleBase.h"

class StressTest : public SampleBase
{
    URHO3D_OBJECT(StressTest, SampleBase);

public:
    StressTest(Context* context);
    ~StressTest();

    void SampleStart() override;
    void Update(float timeStep) override;
};