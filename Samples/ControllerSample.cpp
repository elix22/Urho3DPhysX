#include "ControllerSample.h"
#include <PhysXScene.h>
#include <GroundPlane.h>
#include <StaticBody.h>
#include <DynamicBody.h>
#include <CollisionShape.h>
#include <PhysXMaterial.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/IO/Log.h>

ControllerSample::ControllerSample(Context* context) : SampleBase(context),
dbr_(nullptr),
pxScene_(nullptr),
controller_(nullptr),
anim_(nullptr),
standingHeight_(1.0f),
crouchingHeight_(standingHeight_ / 2),
isCrouching_(false),
moveSpeed_(3.0f),
previousMove_(0),
fixedStep_(0.0f)
{
}

ControllerSample::~ControllerSample()
{
}

void ControllerSample::SampleStart()
{
    CreateSceneAndViewport();
    dbr_ = scene_->CreateComponent<DebugRenderer>();
    using namespace Urho3DPhysX;
    pxScene_ = scene_->CreateComponent<PhysXScene>();
    Node* groundNode = scene_->CreateChild("Ground");
    groundNode->SetScale(1000.0f);
    groundNode->CreateComponent<GroundPlane>();

    for (unsigned i = 0; i < 5; i++)
    {
        Node* bodyNode = scene_->CreateChild();
        bodyNode->SetWorldPosition(Vector3::FORWARD * i * 5 + Vector3(0.0f, -1, 0.0f));
        bodyNode->CreateComponent<StaticBody>();
        auto* shape = bodyNode->CreateComponent<CollisionShape>();
        shape->SetBox(Vector3(i,i,i));
        bodyNode->Rotate(Quaternion(20.0f, Vector3::RIGHT));
    }

    Node* characterNode = scene_->CreateChild("Character");
    characterNode->SetWorldPosition(Vector3(0, 0.5f, 0));
    Node* modelNode = characterNode->CreateChild("Model");
    AnimatedModel* model = modelNode->CreateComponent<AnimatedModel>();
    model->SetModel(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Jack.mdl"));
    model->SetCastShadows(true);
    anim_ = modelNode->CreateComponent<AnimationController>();

    controller_ = characterNode->CreateComponent<KinematicController>();
    controller_->SetPosition(Vector3(0.0f, 11.9f, 0.0f));
    controller_->SetNonWalkableMode(PREV_CLIMBING_FORCE_SLIDING);
    controller_->UseNodeFromFoot(true);
    controller_->ApplyAttributes();

    Node* ctrl2Node = scene_->CreateChild("C2");
    ctrl2Node->SetWorldPosition(Vector3(-5, 0, 0));
    auto* c2 = ctrl2Node->CreateComponent<KinematicController>();
    c2->SetPosition(Vector3(0, 0.9f, 0.0f));
    c2->ApplyAttributes();

    Node* ctrl3Node = scene_->CreateChild("C2");
    ctrl3Node->SetWorldPosition(Vector3(-4, 0, 0));
    auto* c3 = ctrl3Node->CreateComponent<KinematicController>();
    c3->SetPosition(Vector3(0, 0.9f, 0.0f));
    c3->ApplyAttributes();

    cameraNode_->SetParent(characterNode);
    cameraNode_->SetPosition(Vector3(0.0f, 3.0f, -4.0f));
    cameraNode_->LookAt(characterNode->GetWorldPosition() + Vector3(0.0f, 1.5f, 0.0f));

    pxScene_->SetDebugDrawEnabled(true);
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(ControllerSample, HandlePostRenderUpdate));
    SubscribeToEvent(E_CONTROLLERCOLLISION, URHO3D_HANDLER(ControllerSample, HandleCCTCollision));
    SubscribeToEvent(E_COLLISIONSTART, URHO3D_HANDLER(ControllerSample, HandleDynamicCtrlHit));
}

void ControllerSample::Update(float timeStep)
{
    if (controller_ && anim_)
    {
        static const char* WALKING_ANI = "Models/Jack_Walk.ani";
        float speed = controller_->GetLinearVelocity().Length();
        if (speed > 0.0f)
            anim_->PlayExclusive(WALKING_ANI, 0, true, 0.1f);
        else 
            anim_->Stop(WALKING_ANI, 0.5f);
    }
}

void ControllerSample::FixedUpdate(float timeStep)
{
    fixedStep_ = timeStep;
    if (controller_)
    {
        Vector3 moveDir;
        Node* controllerNode = controller_->GetNode();
        auto* input = GetSubsystem<Input>();
        if (input->GetKeyDown(KEY_W))
            moveDir += controllerNode->GetWorldDirection();
        if (input->GetKeyDown(KEY_S))
            moveDir -= controllerNode->GetWorldDirection();
        if (input->GetKeyDown(KEY_A))
            moveDir -= Quaternion(0.0f, 90.0f, 0.0f) * controllerNode->GetWorldDirection();
        if (input->GetKeyDown(KEY_D))
            moveDir += Quaternion(0.0f, 90.0f, 0.0f) * controllerNode->GetWorldDirection();
        moveDir.y_ = 0.0f;
        int mm = input->GetMouseMoveX();
        if (mm)
        {
            //prevent updating controller position when node gets rotated
            controller_->SetIgnoreNodeDirty(true);
            controllerNode->Rotate(Quaternion((float)mm * 0.4f, Vector3::UP), TS_WORLD);
            controller_->SetIgnoreNodeDirty(false);
        }
        previousMove_ = controller_->MoveWithGravity(moveDir * moveSpeed_ * timeStep, 0.001f, timeStep);
        //recursive controllers pushing
        if (pushQueue_.Size())
        {
            unsigned maxPush = 10;
            while (pushQueue_.Size() && maxPush)
            {
                const ControllerPushData& d = pushQueue_.Back();
                pushQueue_.Pop();
                d.controller_->MoveWithGravity(d.direction_ * timeStep, 0.0f, timeStep);
                maxPush--;
            }
            pushQueue_.Clear();
        }
    }
}

void ControllerSample::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    if (dbr_ && pxScene_)
    {
        pxScene_->DrawDebugGeometry(dbr_);
        if (controller_)
            controller_->DrawDebugGeometry(dbr_);
    }
}

void ControllerSample::HandleCCTCollision(StringHash eventType, VariantMap& eventData)
{
    using namespace ControllerCollision;
    KinematicController* other = static_cast<KinematicController*>(eventData[P_OTHER_CONTROLLER].GetPtr());
    if (other)// && other != controller_)
    {
        //push other controller - since this is called during move(), cache the result and call it after move is done
        pushQueue_.Push(ControllerPushData(other, -eventData[P_NORMAL].GetVector3()));        
    }
}

void ControllerSample::HandleDynamicCtrlHit(StringHash eventType, VariantMap& eventData)
{
    using namespace CollisionStart;
    if (eventData[P_CONTROLLERCOLLISION].GetBool())
    {
        DynamicBody* body = static_cast<DynamicBody*>(eventData[P_ACTOR].GetPtr());
        KinematicController* ctrl = static_cast<KinematicController*>(eventData[P_CONTROLLER].GetPtr());
        ctrl->MoveWithGravity(body->GetLinearVelocity() * 5 * fixedStep_, 0.0f, fixedStep_);
    }
}

void ControllerSample::OnKeyUp(Key key)
{
    SampleBase::OnKeyUp(key);
    if (controller_)
    {
        switch (key)
        {
        case(KEY_Q):
        {
            ControllerType currentType = controller_->GetControllerType();
            controller_->SetControllerType(currentType == CAPSULE_CONTROLLER ? BOX_CONTROLLER : CAPSULE_CONTROLLER);
            controller_->ApplyAttributes();
            break;
        }
        case(KEY_C):
            isCrouching_ = !isCrouching_;
            controller_->Resize(isCrouching_ ? crouchingHeight_ : standingHeight_);
            break;
        case(KEY_3):
            controller_->GetNode()->SetWorldPosition(Vector3(0, 10, 0));
        default:
            break;
        }
    }
}
