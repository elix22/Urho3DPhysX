#pragma once
#include "PhysXUtils.h"
#include <Urho3D/Core/Object.h>
#include <foundation/PxErrorCallback.h>
#include <PxSimulationEventCallback.h>
#include <PxBroadPhase.h>
#include <characterkinematic/PxController.h>

using namespace physx;

URHO3D_EVENT(E_PHYSXERROR, PhysXError)
{
    URHO3D_PARAM(P_CODE, Code);
    URHO3D_PARAM(P_MESSAGE, Message);
    URHO3D_PARAM(P_FILE, File);
    URHO3D_PARAM(P_LINE, Line);
}

URHO3D_EVENT(E_PX_PRESIMULATION, PhysXPreSimulation)
{
    URHO3D_PARAM(P_PHYSX_SCENE, PhysXScene);
    URHO3D_PARAM(P_TIMESTEP, TimeStep);
}

URHO3D_EVENT(E_COLLISIONSTART, CollisionStart)
{
    URHO3D_PARAM(P_PHYSX_SCENE, PhysXScene);
    URHO3D_PARAM(P_ACTOR, Actor);
    URHO3D_PARAM(P_OTHERACTOR, OtherActor);
    URHO3D_PARAM(P_CONTROLLER, Controller);
    URHO3D_PARAM(P_CONTROLLERCOLLISION, ControllerCollision);
}

URHO3D_EVENT(E_COLLISION, Collision)
{
    URHO3D_PARAM(P_PHYSX_SCENE, PhysXScene);
    URHO3D_PARAM(P_ACTOR, Actor);
    URHO3D_PARAM(P_OTHERACTOR, OtherActor);
    URHO3D_PARAM(P_CONTROLLER, Controller);
    URHO3D_PARAM(P_CONTROLLERCOLLISION, ControllerCollision);
}

URHO3D_EVENT(E_COLLISIONEND, CollisionEnd)
{
    URHO3D_PARAM(P_PHYSX_SCENE, PhysXScene);
    URHO3D_PARAM(P_ACTOR, Actor);
    URHO3D_PARAM(P_OTHERACTOR, OtherActor);
    URHO3D_PARAM(P_CONTROLLER, Controller);
    URHO3D_PARAM(P_CONTROLLERCOLLISION, ControllerCollision);
}

URHO3D_EVENT(E_TRIGGERENTER, TriggerEnter)
{
    URHO3D_PARAM(P_PHYSX_SCENE, PhysXScene);
    URHO3D_PARAM(P_SHAPE, Shape);
    URHO3D_PARAM(P_ACTOR, Actor);
    URHO3D_PARAM(P_OTHERSHAPE, OtherShape);
    URHO3D_PARAM(P_OTHERACTOR, OtherActor);
}

URHO3D_EVENT(E_TRIGGERLEAVE, TriggerLeave)
{
    URHO3D_PARAM(P_PHYSX_SCENE, PhysXScene);
    URHO3D_PARAM(P_SHAPE, Shape);
    URHO3D_PARAM(P_ACTOR, Actor);
    URHO3D_PARAM(P_OTHERSHAPE, OtherShape);
    URHO3D_PARAM(P_OTHERACTOR, OtherActor);
}

URHO3D_EVENT(E_CONTROLLERCOLLISION, ControllerCollision)
{
    URHO3D_PARAM(P_PHYSX_SCENE, PhysXScene);
    URHO3D_PARAM(P_CONTROLLER, Controller);
    URHO3D_PARAM(P_OTHER_CONTROLLER, OtherController);
    URHO3D_PARAM(P_POSITION, Position);
    URHO3D_PARAM(P_NORMAL, Normal);
    URHO3D_PARAM(P_DIR, Dir);
    URHO3D_PARAM(P_LENGTH, Length);
}

URHO3D_EVENT(E_SHAPECOLLISION, ShapeCollision)
{
    URHO3D_PARAM(P_PHYSX_SCENE, PhysXScene);
    URHO3D_PARAM(P_CONTROLLER, Controller);
    URHO3D_PARAM(P_SHAPE, Shape);
    URHO3D_PARAM(P_ACTOR, Actor);
    URHO3D_PARAM(P_POSITION, Position);
    URHO3D_PARAM(P_NORMAL, Normal);
    URHO3D_PARAM(P_DIR, Dir);
    URHO3D_PARAM(P_LENGTH, Length);
}

namespace Urho3DPhysX
{
    class Physics;
    class PhysXScene;
    class KinematicController;

    class ErrorCallback : public PxErrorCallback
    {
    public:
        ErrorCallback(Physics* physics);
        ~ErrorCallback();

        void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override;

    private:
        Physics* physics_;
    };

    class SimulationEventCallback : public PxSimulationEventCallback
    {
    public:
        SimulationEventCallback(PhysXScene* scene);
        ~SimulationEventCallback();

        void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) override;
        void onWake(PxActor** actors, PxU32 count) override;
        void onSleep(PxActor** actors, PxU32 count) override;
        void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override;
        void onTrigger(PxTriggerPair* pairs, PxU32 count) override;
        void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) override;

    private:
        PhysXScene* scene_;
    };

    class BroadPhaseCallback : public PxBroadPhaseCallback
    {
    public:
        BroadPhaseCallback(PhysXScene* scene);
        ~BroadPhaseCallback();

        void onObjectOutOfBounds(PxShape& shape, PxActor& actor) override;
        void onObjectOutOfBounds(PxAggregate& aggregate) override;

    private:
        PhysXScene* scene_;
    };

    class ControllerHitCallback : public PxUserControllerHitReport
    {
    public:
        ControllerHitCallback();
        ~ControllerHitCallback();

        void onShapeHit(const PxControllerShapeHit& hit) override;
        void onControllerHit(const PxControllersHit& hit) override;
        void onObstacleHit(const PxControllerObstacleHit& hit) override;
    };
}