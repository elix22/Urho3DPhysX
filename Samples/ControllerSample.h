#pragma once

#include "SampleBase.h"
#include <KinematicController.h>

namespace Urho3D
{
    class DebugRenderer;
    class AnimationController;
}

namespace Urho3DPhysX
{
    class PhysXScene;


    class NoCollisionCallback : public PxControllerFilterCallback
    {
    public:
        bool filter(const PxController& a, const PxController& b) override { return false; }
    };
}
using namespace Urho3DPhysX;

struct ControllerPushData
{
    ControllerPushData() :
        controller_(nullptr),
        direction_(Vector3::ZERO)
    {}
    ControllerPushData(KinematicController* c, Vector3 dir) :
        controller_(c),
        direction_(dir)
    {}
    KinematicController* controller_;
    Vector3 direction_;
};

class ControllerSample : public SampleBase
{
    URHO3D_OBJECT(ControllerSample, SampleBase);

public:
    ControllerSample(Context* context);
    ~ControllerSample();

    void SampleStart() override;
    void Update(float timeStep) override;
    void FixedUpdate(float timeStep) override;

private:
    void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData);
    void HandleCCTCollision(StringHash eventType, VariantMap& eventData);
    void HandleDynamicCtrlHit(StringHash eventType, VariantMap& eventData);
    void OnKeyUp(Key key) override;
    WeakPtr<KinematicController> controller_;
    WeakPtr<AnimationController> anim_;
    PhysXScene* pxScene_;
    DebugRenderer* dbr_;
    float standingHeight_;
    float crouchingHeight_;
    float moveSpeed_;
    bool isCrouching_;
    unsigned char previousMove_;
    NoCollisionCallback noCollisionCallback_;
    Vector<ControllerPushData> pushQueue_;
    float fixedStep_;
};
