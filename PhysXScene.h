#pragma once
#include "PhysXEvents.h"
#include <Urho3D/Scene/Component.h>
#include <Urho3D/Math/Ray.h>
#include <PxScene.h>

using namespace Urho3D;
using namespace physx;

namespace Urho3DPhysX
{
    class RigidActor;
    class RigidBody;
    class CollisionShape;
    class Joint;

    struct CollisionData
    {
        CollisionData() :
            actorA_(nullptr),
            actorB_(nullptr),
            eventType_(E_COLLISION)
        {
        }
        WeakPtr<RigidActor> actorA_;
        WeakPtr<RigidActor> actorB_;
        StringHash eventType_;
    };

    struct TriggerData
    {
        TriggerData() :
            trigger_(nullptr),
            triggerActor_(nullptr),
            otherShape_(nullptr),
            otherActor_(nullptr)
        {

        }
        WeakPtr<CollisionShape> trigger_;
        WeakPtr<RigidActor> triggerActor_;
        WeakPtr<CollisionShape> otherShape_;
        WeakPtr<RigidActor> otherActor_;
        StringHash eventType_;
    };

    struct __declspec(dllexport) PhysXRaycastResult
    {
        RigidActor* actor_;
        CollisionShape* shape_;
        Vector3 position_;
        Vector3 normal_;
        float distance_;
    };

    class __declspec(dllexport) PhysXScene : public Component
    {
        URHO3D_OBJECT(PhysXScene, Component);
        friend class SimulationEventCallback;
    public:
        PhysXScene(Context* context);
        ~PhysXScene();

        static void RegisterObject(Context* context);

        void DrawDebugGeometry(DebugRenderer* debug, bool depthTest = true) override;

        void Update(float timeStep);
        ///
        void AddActor(RigidActor* actor);
        ///Remove actor from scene, this will NOT reset scene pointer in actor - use RigidActor::RemoveFromScene instead.
        void RemoveActor(RigidActor* actor);
        ///
        bool Raycast(PODVector<PhysXRaycastResult>& results, const Ray& ray, float maxDistance, unsigned mask);
        ///
        bool RaycastSingle(PhysXRaycastResult& result, const Ray& ray, float maxDistance, unsigned mask);
        ///
        bool SphereCast(PODVector<PhysXRaycastResult>& results, const Ray& ray, float radius, float maxDistance, unsigned mask);
        ///
        bool SphereCastSingle(PhysXRaycastResult& result, const Ray& ray, float radius, float maxDistance, unsigned mask);
        ///
        bool BoxCast(PODVector<PhysXRaycastResult>& results, const Ray& ray, const Vector3& size, const Quaternion& rotation, float maxDistance, unsigned mask);
        ///
        bool BoxCastSingle(PhysXRaycastResult& result, const Ray& ray, const Vector3& size, const Quaternion& rotation, float maxDistance, unsigned mask);
        ///
        bool CapsuleCast(PODVector<PhysXRaycastResult>& results, const Ray& ray, float radius, float height, const Quaternion& rotation, float maxDistance, unsigned mask);
        ///
        bool CapsuleCastSingle(PhysXRaycastResult& result, const Ray& ray, float radius, float height, const Quaternion& rotation, float maxDistance, unsigned mask);
        ///
        void SetDebugDrawEnabled(bool enable);
        ///
        bool IsDebugDrawEnabled() const { return debugDrawEnabled_; }
        ///
        void SetGravity(const Vector3& gravity);
        ///
        const Vector3& GetGravity() const { return gravity_; }
        ///
        void SetFPS(float value) { fps_ = value; }
        ///
        float GetFPS() const { return fps_; }
        ///
        void SetMaxSubsteps(int value) { maxSubsteps_ = value; }
        ///
        int GetMaxSubsteps() const { return maxSubsteps_; }
        ///
        bool IsSimulating() const { return isSimulating_; }
        ///
        void SetProcessSimulationEvents(bool process);
        ///
        bool IsProcessingSimulationEvents() const { return processSimEvents_; }
    private:
        void HandleSceneSubsystemUpdate(StringHash eventType, VariantMap& eventData);
        void OnSceneSet(Scene* scene) override;
        ///
        void AddCollision(const PxContactPairHeader & pairHeader, const PxContactPair * pairs, PxU32 nbPairs);
        ///
        void AddTriggerEvents(PxTriggerPair* pair, unsigned numPairs);
        ///
        void ProcessTriggers();
        ///
        void ProcessCollisions();
        ///
        bool Sweep(PODVector<PhysXRaycastResult>& results, const Ray& ray, const Quaternion& rotation, const PxGeometry& geometry, float maxDistance, unsigned mask);
        ///
        bool SweepSingle(PhysXRaycastResult& result, const Ray& ray, const Quaternion& rotation, const PxGeometry& geometry, float maxDistance, unsigned mask);
        ///
        void ReleaseScene();
        //temp
        void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData);
        PxScene* pxScene_;
        SimulationEventCallback simulationEventCallback_;
        BroadPhaseCallback broadPhaseCallback_;
        bool isSimulating_;
        float fps_;
        int maxSubsteps_;
        float timeAcc_;
        Vector3 gravity_;
        bool processSimEvents_;
        Vector<CollisionData> collisions_;
        Vector<TriggerData> triggers_;
        VariantMap triggersDataMap_;
        VariantMap collisionDataMap_;
        bool debugDrawEnabled_;
        PODVector<RigidActor*> rigidActors_;
    };
}
