#include "EventsSample.h"
#include "PhysXEvents.h"
#include <GroundPlane.h>
#include <StaticBody.h>
#include <DynamicBody.h>
#include <CollisionShape.h>
#include <Urho3D/IO/Log.h>

using namespace Urho3DPhysX;
EventsSample::EventsSample(Context * context) : SampleBase(context)
{
}

EventsSample::~EventsSample()
{
}

void EventsSample::SampleStart()
{
    CreateSceneAndViewport();
    Node* ground = scene_->CreateChild("Ground");
    ground->SetScale(1000.0f);
    GroundPlane* groundPlane = ground->CreateComponent<GroundPlane>();

    Node* staticBNode = CreateCube(Vector3::UP * 10.0f);
    StaticBody* staticBody = staticBNode->CreateComponent<StaticBody>();
    CollisionShape* staticShape = staticBNode->CreateComponent<CollisionShape>();
    staticShape->SetBox(Vector3(1.5f, 1.5f, 1.5f));
    staticShape->SetTrigger(true);
    CollisionShape* shape2 = staticBNode->CreateComponent<CollisionShape>();
    shape2->SetBox();

    SubscribeToEvent(E_COLLISIONSTART, URHO3D_HANDLER(EventsSample, HandleCollisionStart));
    SubscribeToEvent(E_COLLISION, URHO3D_HANDLER(EventsSample, HandleCollision));
    SubscribeToEvent(E_COLLISIONEND, URHO3D_HANDLER(EventsSample, HandleCollisionEnd));
    SubscribeToEvent(E_TRIGGERENTER, URHO3D_HANDLER(EventsSample, HandleTriggerEnter));
    SubscribeToEvent(E_TRIGGERLEAVE, URHO3D_HANDLER(EventsSample, HandleTriggerLeave));
}

void EventsSample::Update(float timeStep)
{
    MoveFreeCamera(timeStep);
}

void EventsSample::HandleCollisionStart(StringHash eventType, VariantMap & eventData)
{

    using namespace Collision;
    RigidActor* sender = static_cast<RigidActor*>(eventData[P_ACTOR].GetPtr());
    RigidActor* other = static_cast<RigidActor*>(eventData[P_OTHERACTOR].GetPtr());
    if (sender && other)
    {
        Vector3 scale = other->GetNode()->GetScale();
        GroundPlane* g = sender->GetComponent<GroundPlane>();
        if (!g)
            g = other->GetComponent<GroundPlane>();
        if (g)
            return;
    }
}

void EventsSample::HandleCollisionEnd(StringHash eventType, VariantMap & eventData)
{

    using namespace Collision;
    RigidActor* sender = static_cast<RigidActor*>(eventData[P_ACTOR].GetPtr());
    RigidActor* other = static_cast<RigidActor*>(eventData[P_OTHERACTOR].GetPtr());
    if (sender)
    {
        //Vector3 scale = other->GetNode()->GetScale();
        GroundPlane* g = sender->GetComponent<GroundPlane>();
        if (!g && other)
            g = other->GetComponent<GroundPlane>();
        if (g)
            return;
    }
}

void EventsSample::HandleCollision(StringHash eventType, VariantMap & eventData)
{
    using namespace Collision;
    RigidActor* sender = static_cast<RigidActor*>(eventData[P_ACTOR].GetPtr());
    RigidActor* other = static_cast<RigidActor*>(eventData[P_OTHERACTOR].GetPtr());
    if (sender && other)
    {
        Vector3 scale = other->GetNode()->GetScale();
        GroundPlane* g = sender->GetComponent<GroundPlane>();
        if (!g)
            g = other->GetComponent<GroundPlane>();
        if (g)
            return;
    }
}

void EventsSample::HandleTriggerEnter(StringHash eventType, VariantMap & eventData)
{
    using namespace TriggerEnter;
    DynamicBody* body = static_cast<DynamicBody*>(eventData[P_OTHERACTOR].GetPtr());
    if (body)
    {
        body->ApplyImpulse(Vector3(0.0f, 5.0f, 0.0f));
    }
}

void EventsSample::HandleTriggerLeave(StringHash eventType, VariantMap & eventData)
{
    using namespace TriggerLeave;
    DynamicBody* body = static_cast<DynamicBody*>(eventData[P_OTHERACTOR].GetPtr());
    RigidActor* trigger = static_cast<RigidActor*>(eventData[P_ACTOR].GetPtr());
    if (body && trigger)
    {
        if (body->GetNode() == trigger->GetNode())
            return;
        GroundPlane* g = body->GetComponent<GroundPlane>();
        if (!g)
            g = body->GetComponent<GroundPlane>();
        if (g)
            return;
        //body->ApplyTorque(Vector3(0.0f, 145.0f, 0.0f));
        body->ApplyTorqueImpulse(-body->GetNode()->GetWorldDirection() * 100);
    }
}
