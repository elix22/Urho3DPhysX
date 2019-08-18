#pragma once

#include "SampleBase.h"

class EventsSample : public SampleBase
{
    URHO3D_OBJECT(EventsSample, SampleBase);
public:
    EventsSample(Context* context);
    ~EventsSample();

    void SampleStart() override;
    void Update(float timeStep) override;

private:
    void HandleCollisionStart(StringHash eventType, VariantMap& eventData);
    void HandleCollisionEnd(StringHash eventType, VariantMap& eventData);
    void HandleCollision(StringHash eventType, VariantMap& eventData);
    void HandleTriggerEnter(StringHash eventType, VariantMap& eventData);
    void HandleTriggerLeave(StringHash eventType, VariantMap& eventData);
};
