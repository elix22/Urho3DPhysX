#include "PhysXEvents.h"
#include "Physics.h"
#include "PhysXScene.h"
#include "RigidActor.h"
#include "KinematicController.h"
#include <Urho3D/IO/Log.h>

Urho3DPhysX::ErrorCallback::ErrorCallback(Physics* physics) :
physics_(physics)
{
}

Urho3DPhysX::ErrorCallback::~ErrorCallback()
{
}

void Urho3DPhysX::ErrorCallback::reportError(PxErrorCode::Enum code, const char * message, const char * file, int line)
{
    if (physics_)
        physics_->SendErrorEvent(code, message, file, line);
}

Urho3DPhysX::SimulationEventCallback::SimulationEventCallback(PhysXScene * scene) :
    scene_(scene)
{
}

Urho3DPhysX::SimulationEventCallback::~SimulationEventCallback()
{
}

void Urho3DPhysX::SimulationEventCallback::onConstraintBreak(PxConstraintInfo * constraints, PxU32 count)
{
}

void Urho3DPhysX::SimulationEventCallback::onWake(PxActor ** actors, PxU32 count)
{
}

void Urho3DPhysX::SimulationEventCallback::onSleep(PxActor ** actors, PxU32 count)
{
}

void Urho3DPhysX::SimulationEventCallback::onContact(const PxContactPairHeader & pairHeader, const PxContactPair * pairs, PxU32 nbPairs)
{
    if (scene_)
    {
        scene_->AddCollision(pairHeader, pairs, nbPairs);
    }
}

void Urho3DPhysX::SimulationEventCallback::onTrigger(PxTriggerPair * pairs, PxU32 count)
{
    if (scene_)
    {
        scene_->AddTriggerEvents(pairs, count);
    }
}

void Urho3DPhysX::SimulationEventCallback::onAdvance(const PxRigidBody * const * bodyBuffer, const PxTransform * poseBuffer, const PxU32 count)
{
}

Urho3DPhysX::BroadPhaseCallback::BroadPhaseCallback(PhysXScene * scene) : 
    scene_(scene)
{
}

Urho3DPhysX::BroadPhaseCallback::~BroadPhaseCallback()
{
}

void Urho3DPhysX::BroadPhaseCallback::onObjectOutOfBounds(PxShape & shape, PxActor & actor)
{
}

void Urho3DPhysX::BroadPhaseCallback::onObjectOutOfBounds(PxAggregate & aggregate)
{
}

Urho3DPhysX::ControllerHitCallback::ControllerHitCallback()
{

}

Urho3DPhysX::ControllerHitCallback::~ControllerHitCallback()
{
}

void Urho3DPhysX::ControllerHitCallback::onShapeHit(const PxControllerShapeHit& hit)
{
    static_cast<KinematicController*>(hit.controller->getUserData())->OnShapeHit(hit);
}

void Urho3DPhysX::ControllerHitCallback::onControllerHit(const PxControllersHit& hit)
{
    static_cast<KinematicController*>(hit.controller->getUserData())->OnControllerHit(hit);
}

void Urho3DPhysX::ControllerHitCallback::onObstacleHit(const PxControllerObstacleHit& hit)
{
}
