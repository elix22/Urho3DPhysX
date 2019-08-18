#include "RaycastSample.h"
#include <PhysXScene.h>
#include <StaticBody.h>
#include <CollisionShape.h>
#include <PhysXEvents.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Graphics/DebugRenderer.h>

using namespace Urho3DPhysX;
static const unsigned LAYER_1 = 0x1;
static const unsigned LAYER_2 = 0x2;
static const unsigned LAYER_3 = 0x4;
static const unsigned LAYERS1_2 = LAYER_1 | LAYER_2;

RaycastSample::RaycastSample(Context * context) : SampleBase(context),
dbr_(nullptr),
pxScene_(nullptr),
sweepRadius_(1.0f)
{
}

RaycastSample::~RaycastSample()
{
}

void RaycastSample::SampleStart()
{
    CreateSceneAndViewport();
    pxScene_ = scene_->CreateComponent<PhysXScene>();
    dbr_ = scene_->CreateComponent<DebugRenderer>();
    //create cubes and set collision layers
    for (int x = -10; x <= 10; x += 10)
    {
        for (int z = -10; z <= 10; z += 10)
        {
            if (x == 0 && z == 0)
                continue;
            for (unsigned i = 0; i < 20; ++i)
            {
                Node* node = CreateCube(Vector3(x, i * 2, z));
                StaticBody* body = node->CreateComponent<StaticBody>();
                CollisionShape* shape = node->CreateComponent<CollisionShape>();
                shape->SetCollisionLayer(i % 2 == 0 ? LAYER_2 : LAYER_1);
            }
        }
    }
    //create trigger plane to destroy falling objects
    Node* triggerNode = scene_->CreateChild();
    triggerNode->SetPosition(Vector3(0.0f, -25.0f, 0.0f));
    StaticBody* triggerBody = triggerNode->CreateComponent<StaticBody>();
    CollisionShape* trigger = triggerNode->CreateComponent<CollisionShape>();
    trigger->SetShapeType(PLANE_SHAPE);
    trigger->SetCollisionLayer(LAYER_3);
    trigger->SetTrigger(true);
    SubscribeToEvent(trigger, E_TRIGGERENTER, URHO3D_HANDLER(RaycastSample, HandleTriggerEnter));
}

void RaycastSample::Update(float timeStep)
{
    MoveFreeCamera(timeStep);
    if (dbr_ && pxScene_)
    {
        //camera ray
        {
            PhysXRaycastResult result;
            if (pxScene_->RaycastSingle(result, Ray(cameraNode_->GetWorldPosition(), cameraNode_->GetWorldDirection()), 100.0f, LAYERS1_2))
            {
                dbr_->AddSphere(Sphere(result.position_, 0.2f), Color::RED);
            }
        }
        //Raycast single
        {
            PhysXRaycastResult result;
            Vector3 startPos(-10.0f, -5.0f, -10.0f);
            Color rayColor;
            if (pxScene_->RaycastSingle(result, Ray(startPos, Vector3::UP), 50.0f, LAYER_1))
            {
                dbr_->AddSphere(Sphere(result.position_, 0.3f), Color::GREEN, true);
                rayColor = Color::BLUE;
            }
            else
            {
                rayColor = Color::RED;
            }
            dbr_->AddLine(startPos, startPos + Vector3(0.0f, 50.0f, 0.0f), rayColor, true);
            
        }
        //Raycast
        {
            PODVector<PhysXRaycastResult> results;
            Vector3 startPos(0.0f, -5.0f, -10.0f);
            Color rayColor;
            if (pxScene_->Raycast(results, Ray(startPos, Vector3::UP), 50.0f, LAYER_2))
            {
                for(auto r : results)
                    dbr_->AddSphere(Sphere(r.position_, 0.3f), Color::GREEN, true);
                rayColor = Color::BLUE;
            }
            else
            {
                rayColor = Color::RED;
            }
            dbr_->AddLine(startPos, startPos + Vector3(0.0f, 50.0f, 0.0f), rayColor, true);

        }
        //Sphere cast single
        {
            PhysXRaycastResult result;
            Vector3 startPos(10.75f, -5.0f, -10.75f);
            Color rayColor;
            if (pxScene_->SphereCastSingle(result, Ray(startPos, Vector3::UP), 2.0f, 50.0f, LAYER_2))
            {
                dbr_->AddSphere(Sphere(result.position_, 0.3f), Color::GREEN, true);
                rayColor = Color::BLUE;
            }
            else
            {
                rayColor = Color::RED;
            }
            dbr_->AddLine(startPos, startPos + Vector3(0.0f, 50.0f, 0.0f), rayColor, true);
        }
        //Sphere cast
        {
            PODVector<PhysXRaycastResult> results;
            Vector3 startPos(10.75f, -5.0f, 0.75f);
            Color rayColor;
            if (pxScene_->SphereCast(results, Ray(startPos, Vector3::UP), 1.0f, 50.0f, LAYER_1))
            {
                for (auto r : results)
                    dbr_->AddSphere(Sphere(r.position_, 0.3f), Color::GREEN, true);
                rayColor = Color::BLUE;
            }
            else
            {
                rayColor = Color::RED;
            }
            dbr_->AddLine(startPos, startPos + Vector3(0.0f, 50.0f, 0.0f), rayColor, true);
        }
        //Sphere cast - all layers
        {
            PODVector<PhysXRaycastResult> results;
            Vector3 startPos(10.75f, -5.0f, 10.75f);
            Color rayColor;
            if (pxScene_->SphereCast(results, Ray(startPos, Vector3::UP), 1.0f, 50.0f, LAYERS1_2))
            {
                for (auto r : results)
                    dbr_->AddSphere(Sphere(r.position_, 0.3f), Color::GREEN, true);
                rayColor = Color::BLUE;
            }
            else
            {
                rayColor = Color::RED;
            }
            dbr_->AddLine(startPos, startPos + Vector3(0.0f, 50.0f, 0.0f), rayColor, true);
        }

    }
}

void RaycastSample::HandleTriggerEnter(StringHash eventType, VariantMap & eventData)
{
    using namespace TriggerEnter;
    RigidActor* actor = static_cast<RigidActor*>(eventData[P_OTHERACTOR].GetPtr());
    if (actor)
        actor->GetNode()->Remove();
}
