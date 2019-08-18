#pragma once

#include "SampleBase.h"

class SimpleScene : public SampleBase
{
    URHO3D_OBJECT(SimpleScene, SampleBase);

public:
    SimpleScene(Context* context);
    ~SimpleScene();

    void SampleStart() override;
    void Update(float timeStep) override;
};
