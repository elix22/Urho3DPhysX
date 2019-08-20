#include "PhysXScene.h"
#include "Physics.h"
#include "RigidActor.h"
#include "PhysXUtils.h"
#include "DynamicBody.h"
#include "CollisionShape.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Profiler.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/IO/Log.h>

namespace Urho3DPhysX
{
    static const float DEF_FPS = 60.0f;
    static const Vector3 DEF_GRAVITY = Vector3(0.0f, -9.81f, 0.0f);
    static const unsigned MAX_RAYCAST_HITS = 64;
    //
    static PxFilterFlags physxSceneFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
        PxFilterObjectAttributes attributes1, PxFilterData filterData1,
        PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
    {
        //triggers
        if (PxFilterObjectIsTrigger(attributes0) && PxFilterObjectIsTrigger(attributes1))
        {
            pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
            return PxFilterFlag::eDEFAULT;
        }

        if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
        {
            pairFlags = PxPairFlag::eCONTACT_DEFAULT //default: eSOLVE_CONTACT | eDETECT_DISCRETE_CONTACT
                | PxPairFlag::eTRIGGER_DEFAULT  //notify contact start and end
                | PxPairFlag::eNOTIFY_TOUCH_PERSISTS
                | PxPairFlag::eMODIFY_CONTACTS; //for later use (impl modify contact callback)
        }
        else
        {
            return PxFilterFlag::eSUPPRESS;
        }

        return PxFilterFlag::eDEFAULT;
    }

    static bool ComparePhysXRaycastResults(const PhysXRaycastResult& lhs, const PhysXRaycastResult& rhs)
    {
        return lhs.distance_ < rhs.distance_;
    }

    static const PxHitFlags DEF_RAYCAST_FLAGS = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
}

Urho3DPhysX::PhysXScene::PhysXScene(Context * context) : Component(context),
pxScene_(nullptr),
fps_(DEF_FPS),
gravity_(DEF_GRAVITY),
broadPhaseCallback_(this),
simulationEventCallback_(this),
debugDrawEnabled_(false),
maxSubsteps_(0),
timeAcc_(0.0f),
processSimEvents_(true)
{
}

Urho3DPhysX::PhysXScene::~PhysXScene()
{
    ReleaseScene();
}

void Urho3DPhysX::PhysXScene::RegisterObject(Context * context)
{
    context->RegisterFactory<PhysXScene>("PhysX");

    URHO3D_ACCESSOR_ATTRIBUTE("Enable debug drawing", IsDebugDrawEnabled, SetDebugDrawEnabled, bool, false, AM_EDIT);
    URHO3D_ACCESSOR_ATTRIBUTE("Gravity", GetGravity, SetGravity, Vector3, DEF_GRAVITY, AM_DEFAULT);
    URHO3D_ATTRIBUTE("FPS", float, fps_, DEF_FPS, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Max substeps", int, maxSubsteps_, 0, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Sim events enabled", IsProcessingSimulationEvents, SetProcessSimulationEvents, bool, true, AM_DEFAULT);
}

void Urho3DPhysX::PhysXScene::DrawDebugGeometry(DebugRenderer * debug, bool depthTest)
{
    URHO3D_PROFILE(PhysXDebug);
    if (!debugDrawEnabled_)
        return;
    if (debug)
    {
        const PxRenderBuffer& rb = pxScene_->getRenderBuffer();
        for (PxU32 i = 0; i < rb.getNbLines(); i++)
        {
            const PxDebugLine& line = rb.getLines()[i];
            debug->AddLine(ToVector3(line.pos0), ToVector3(line.pos1), line.color0);
        }
        for (PxU32 i = 0; i < rb.getNbTriangles(); i++)
        {
            const PxDebugTriangle& t = rb.getTriangles()[i];
            debug->AddTriangle(ToVector3(t.pos0), ToVector3(t.pos1), ToVector3(t.pos2), t.color0);
        }
        for (PxU32 i = 0; i < rb.getNbPoints(); ++i)
        {
            const PxDebugPoint& p = rb.getPoints()[i];
            Color c;
            c.FromUInt(p.color);
            debug->AddSphere(Sphere(ToVector3(p.pos), 0.5f), c);
        }
    }
}

void Urho3DPhysX::PhysXScene::Update(float timeStep)
{
    URHO3D_PROFILE(PhysXUpdate);
    if (!pxScene_)
        return;
    float internalTimeStep = 1.0f / fps_;
    int maxSubsteps = (int)(timeStep * fps_) + 1;
    if (maxSubsteps_ < 0)
    {
        internalTimeStep = timeStep;
        maxSubsteps = 1;
    }
    else if (maxSubsteps_ > 0)
        maxSubsteps = Min(maxSubsteps, maxSubsteps_);

    isSimulating_ = true;
    timeAcc_ += timeStep;
    while (timeAcc_ >= internalTimeStep && maxSubsteps > 0)
    {
        pxScene_->simulate(internalTimeStep);
        timeAcc_ -= internalTimeStep;
        --maxSubsteps;
        if (!pxScene_->fetchResults(true))
        {
            //__debugbreak();
        }
    }

    isSimulating_ = false;
    PxU32 numActiveActors;
    PxActor** acitveActors = pxScene_->getActiveActors(numActiveActors);
    for (PxU32 i = 0; i < numActiveActors; i++)
    {
        RigidActor* a = static_cast<RigidActor*>(acitveActors[i]->userData);
        if (a)
        {
            a->ApplyWorldTransformFromActor();
        }
    }
    ProcessTriggers();
    ProcessCollisions();
}

void Urho3DPhysX::PhysXScene::AddActor(RigidActor * actor)
{
    rigidActors_.Push(actor);
    pxScene_->addActor(*actor->GetActor());
}

bool Urho3DPhysX::PhysXScene::Raycast(PODVector<PhysXRaycastResult>& results, const Ray & ray, float maxDistance, unsigned mask)
{
    URHO3D_PROFILE(PhysXRaycast);
    if (pxScene_)
    {
        PxRaycastBufferN<MAX_RAYCAST_HITS> buffer;
        PxQueryFilterData filter;
        filter.data.word0 = mask;
        if (pxScene_->raycast(ToPxVec3(ray.origin_), ToPxVec3(ray.direction_), Clamp(maxDistance, 0.0f, M_INFINITY), buffer, DEF_RAYCAST_FLAGS, filter))
        {
            unsigned numHits = buffer.getNbAnyHits();
            if (numHits)
            {
                results.Clear();
                results.Reserve(numHits);
                for (unsigned i = 0; i < numHits; ++i)
                {
                    const PxRaycastHit& hit = buffer.getAnyHit(i);
                    PhysXRaycastResult result;
                    result.actor_ = static_cast<RigidActor*>(hit.actor->userData);
                    result.shape_ = static_cast<CollisionShape*>(hit.shape->userData);
                    result.distance_ = hit.distance;
                    result.normal_ = ToVector3(hit.normal);
                    result.position_ = ToVector3(hit.position);
                    results.Push(result);
                }
                Sort(results.Begin(), results.End(), ComparePhysXRaycastResults);
                return true;
            }
        }
    }
    return false;
}

bool Urho3DPhysX::PhysXScene::RaycastSingle(PhysXRaycastResult & result, const Ray & ray, float maxDistance, unsigned mask)
{
    URHO3D_PROFILE(PhysXRaycastSingle);
    if (pxScene_)
    {
        PxRaycastBuffer buffer;
        PxQueryFilterData filter;
        filter.data.word0 = mask;
        if (pxScene_->raycast(ToPxVec3(ray.origin_), ToPxVec3(ray.direction_), Clamp(maxDistance, 0.0f, M_INFINITY), buffer, DEF_RAYCAST_FLAGS, filter))
        {
            if (buffer.hasBlock)
            {
                const PxRaycastHit& block = buffer.block;
                result.actor_ = static_cast<RigidActor*>(block.actor->userData);
                result.shape_ = static_cast<CollisionShape*>(block.shape->userData);
                result.distance_ = block.distance;
                result.normal_ = ToVector3(block.normal);
                result.position_ = ToVector3(block.position);
                return true;
            }
        }
    }
    return false;
}

bool Urho3DPhysX::PhysXScene::SphereCast(PODVector<PhysXRaycastResult>& results, const Ray & ray, float radius, float maxDistance, unsigned mask)
{
    return Sweep(results, ray, Quaternion::IDENTITY, PxSphereGeometry(radius), maxDistance, mask);
}

bool Urho3DPhysX::PhysXScene::SphereCastSingle(PhysXRaycastResult & result, const Ray & ray, float radius, float maxDistance, unsigned mask)
{
    return SweepSingle(result, ray, Quaternion::IDENTITY, PxSphereGeometry(radius), maxDistance, mask);
}

bool Urho3DPhysX::PhysXScene::BoxCast(PODVector<PhysXRaycastResult>& results, const Ray & ray, const Vector3 & size, const Quaternion & rotation, float maxDistance, unsigned mask)
{
    return Sweep(results, ray, rotation, PxBoxGeometry(ToPxVec3(size * 0.5f)), maxDistance, mask);
}

bool Urho3DPhysX::PhysXScene::BoxCastSingle(PhysXRaycastResult & result, const Ray & ray, const Vector3 & size, const Quaternion & rotation, float maxDistance, unsigned mask)
{
    return SweepSingle(result, ray, rotation, PxBoxGeometry(ToPxVec3(size * 0.5f)), maxDistance, mask);
}

bool Urho3DPhysX::PhysXScene::CapsuleCast(PODVector<PhysXRaycastResult>& results, const Ray & ray, float radius, float height, const Quaternion & rotation, float maxDistance, unsigned mask)
{
    return Sweep(results, ray, rotation, PxCapsuleGeometry(radius, height * 0.5f), maxDistance, mask);
}

bool Urho3DPhysX::PhysXScene::CapsuleCastSingle(PhysXRaycastResult & result, const Ray & ray, float radius, float height, const Quaternion & rotation, float maxDistance, unsigned mask)
{
    return SweepSingle(result, ray, rotation, PxCapsuleGeometry(radius, height * 0.5f), maxDistance, mask);
}

void Urho3DPhysX::PhysXScene::SetDebugDrawEnabled(bool enable)
{
#ifdef _DEBUG
    if (debugDrawEnabled_ != enable)
    {
        debugDrawEnabled_ = enable;
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eSCALE, debugDrawEnabled_ ? 1.0f : 0.0f);
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_STATIC, 1.0f);
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eBODY_AXES, 1.0f);
        //pxScene_->setVisualizationParameter(PxVisualizationParameter::eWORLD_AXES, 1.0f);
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT, 1.0f);
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eCULL_BOX, 1.0f);
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eMBP_REGIONS, 1.0f);
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
        pxScene_->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
    }
#endif // !_DEBUG
}

void Urho3DPhysX::PhysXScene::SetGravity(const Vector3 & gravity)
{
    if (gravity != gravity_)
    {
        gravity_ = gravity;
        if (pxScene_)
            pxScene_->setGravity(ToPxVec3(gravity_));
    }
}

void Urho3DPhysX::PhysXScene::RemoveActor(RigidActor * actor)
{
    if (actor && pxScene_)
    {
        pxScene_->removeActor(*actor->GetActor());
        rigidActors_.Remove(actor);
    }
}

void Urho3DPhysX::PhysXScene::AddCollision(const PxContactPairHeader & pairHeader, const PxContactPair * pairs, PxU32 nbPairs)
{
    WeakPtr<RigidActor> a(static_cast<RigidActor*>(pairHeader.actors[0]->userData));
    WeakPtr<RigidActor> b(static_cast<RigidActor*>(pairHeader.actors[1]->userData));
    CollisionData data;
    data.actorA_ = a;
    data.actorB_ = b;
    for (unsigned i = 0; i < nbPairs; ++i)
    {
        if (pairs[i].flags & PxContactPairFlag::eACTOR_PAIR_HAS_FIRST_TOUCH)
            data.eventType_ = E_COLLISIONSTART;
        else if (pairs[i].flags & PxContactPairFlag::eACTOR_PAIR_LOST_TOUCH)
            data.eventType_ = E_COLLISIONEND;
    }
    collisions_.Push(data);
}

void Urho3DPhysX::PhysXScene::AddTriggerEvents(PxTriggerPair* pairs, unsigned numPairs)
{
    triggers_.Reserve(numPairs);
    for (unsigned i = 0; i < numPairs; i++)
    {
        TriggerData data;
        const PxTriggerPair& pair = pairs[i];
        bool triggerRemoved = pair.flags & PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER;
        bool otherRemoved = pair.flags & PxTriggerPairFlag::eREMOVED_SHAPE_OTHER;
        data.trigger_ = triggerRemoved ? nullptr : static_cast<CollisionShape*>(pair.triggerShape->userData);
        data.triggerActor_ = triggerRemoved ? nullptr : static_cast<RigidActor*>(pair.triggerActor->userData);
        data.otherShape_ = otherRemoved ? nullptr : static_cast<CollisionShape*>(pair.otherShape->userData);
        data.otherActor_ = otherRemoved ? nullptr : static_cast<RigidActor*>(pair.otherActor->userData);
        //TODO: check if only shape was removed and actor still exists
        if (pair.status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
            data.eventType_ = E_TRIGGERENTER;
        else if (pair.status & PxPairFlag::eNOTIFY_TOUCH_LOST)
            data.eventType_ = E_TRIGGERLEAVE;
        triggers_.Push(data);
    }
}

void Urho3DPhysX::PhysXScene::SetProcessSimulationEvents(bool process)
{
    if (processSimEvents_ != process)
    {
        processSimEvents_ = process;
        if (pxScene_)
        {
            if (process)
                pxScene_->setSimulationEventCallback(&simulationEventCallback_);
            else
                pxScene_->setSimulationEventCallback(nullptr);
        }
    }
}

void Urho3DPhysX::PhysXScene::HandleSceneSubsystemUpdate(StringHash eventType, VariantMap & eventData)
{
    if (!enabled_)
        return;
    Update(eventData[SceneSubsystemUpdate::P_TIMESTEP].GetFloat());
}

void Urho3DPhysX::PhysXScene::OnSceneSet(Scene * scene)
{
    if (scene)
    {
        PxBroadPhaseType::Enum broadPhaseType;
        Variant bptVar = scene->GetVar("BPT");
        if (bptVar.IsEmpty())
        {
            broadPhaseType = GetSubsystem<Physics>()->GetDefBroadPhaseType();
        }
        else
        {
            String type = bptVar.GetString();
            if (type == "SAP")
                broadPhaseType = PxBroadPhaseType::eSAP;
            else if (type == "MBP")
                URHO3D_LOGWARNING("Multibox prunning not supported yet.");//broadPhaseType = PxBroadPhaseType::eMBP;
            else if (type == "ABP")
                broadPhaseType = PxBroadPhaseType::eABP;
            else if (type == "GPU")
                broadPhaseType = PxBroadPhaseType::eGPU;
            else
                broadPhaseType = GetSubsystem<Physics>()->GetDefBroadPhaseType();
        }
        Variant gpuDynamicVar = scene->GetVar("GPUD");
        bool useGPUDynamics = gpuDynamicVar.IsEmpty() ? GetSubsystem<Physics>()->GetDefEnableGPUDynamics() : gpuDynamicVar.GetBool();
        Variant ccdVar = scene->GetVar("CCD");
        bool useCCD = ccdVar.IsEmpty() ? GetSubsystem<Physics>()->GetDefUseCCD() : ccdVar.GetBool();

#ifdef _DEBUG
        String message = "Initializing physx scene with broad phase type: ";
        switch (broadPhaseType)
        {
        case physx::PxBroadPhaseType::eSAP:
            message.Append("3-axes sweep-and-prune, ");
            break;
        case physx::PxBroadPhaseType::eMBP:
            message.Append("Multi box pruning, ");
            break;
        case physx::PxBroadPhaseType::eABP:
            message.Append("Automatic box pruning, ");
            break;
        case physx::PxBroadPhaseType::eGPU:
            message.Append("GPU, ");
            break;
        case physx::PxBroadPhaseType::eLAST:
            break;
        default:
            break;
        }
        message.Append("gpu dynamics ");
        message.Append(useGPUDynamics ? "enabled, " : "disabled, ");
        message.Append("ccd ");
        message.Append(useCCD ? "enabled." : "disabled.");
        URHO3D_LOGDEBUG(message);
#endif // _DEBUG

        auto* physics = GetSubsystem<Physics>();
        auto* px = physics->GetPhysics();
        PxSceneDesc descr(px->getTolerancesScale());
        descr.cpuDispatcher = physics->GetCpuDispatcher();
        descr.cudaContextManager = physics->GetCUDAContextManager();
        descr.gravity = ToPxVec3(gravity_);
        if(processSimEvents_)
            descr.simulationEventCallback = &simulationEventCallback_;
        descr.broadPhaseCallback = &broadPhaseCallback_;
        descr.broadPhaseType = broadPhaseType;
        descr.filterShader = physxSceneFilterShader;

        //flags
        descr.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;
        if(useCCD)
            descr.flags |= PxSceneFlag::eENABLE_CCD;
        if(useGPUDynamics)
            descr.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;

        pxScene_ = px->createScene(descr);
        pxScene_->userData = this;

        SubscribeToEvent(scene, E_SCENESUBSYSTEMUPDATE, URHO3D_HANDLER(PhysXScene, HandleSceneSubsystemUpdate));
        //SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(PhysXScene, HandlePostRenderUpdate));
    }
}

void Urho3DPhysX::PhysXScene::ProcessTriggers()
{
    URHO3D_PROFILE(ProcessTriggers);
    using namespace TriggerEnter;
    if (triggers_.Size())
    {
        for (auto& data : triggers_)
        {
            triggersDataMap_[P_PHYSX_SCENE] = this;
            WeakPtr<CollisionShape> shape = data.trigger_;
            triggersDataMap_[P_SHAPE] = shape;
            triggersDataMap_[P_ACTOR] = data.triggerActor_;
            triggersDataMap_[P_OTHERSHAPE] = data.otherShape_;
            triggersDataMap_[P_OTHERACTOR] = data.otherActor_;
            if (shape)
                shape->SendEvent(data.eventType_, triggersDataMap_);
            else
                SendEvent(data.eventType_, triggersDataMap_);
        }
        triggers_.Clear();
    }
}

void Urho3DPhysX::PhysXScene::ProcessCollisions()
{
    URHO3D_PROFILE(ProcessCollisions);
    using namespace Collision;
    if (collisions_.Size())
    {
        for (auto& data : collisions_)
        {
            WeakPtr<RigidActor> actorA = data.actorA_;
            WeakPtr<RigidActor> actorB = data.actorB_;
            collisionDataMap_[P_PHYSX_SCENE] = this;
            if(actorA)
            {
                collisionDataMap_[P_ACTOR] = actorA;
                collisionDataMap_[P_OTHERACTOR] = actorB ? actorB : nullptr;
                actorA->SendEvent(data.eventType_, collisionDataMap_);
                //if event is collision start, send also normal collision
                if (data.eventType_ == E_COLLISIONSTART)
                    actorA->SendEvent(E_COLLISION, collisionDataMap_);
            }
            if(actorB)
            {
                collisionDataMap_[P_ACTOR] = actorB;
                collisionDataMap_[P_OTHERACTOR] = actorA ? actorA : nullptr;
                actorB->SendEvent(data.eventType_, collisionDataMap_);
                //if event is collision start, send also normal collision
                if (data.eventType_ == E_COLLISIONSTART)
                    actorB->SendEvent(E_COLLISION, collisionDataMap_);
            }
        }
        collisions_.Clear();
    }
}

bool Urho3DPhysX::PhysXScene::Sweep(PODVector<PhysXRaycastResult>& results, const Ray & ray, const Quaternion & rotation, const PxGeometry & geometry, float maxDistance, unsigned mask)
{
    URHO3D_PROFILE(PhysXSweep);
    if (pxScene_)
    {
        PxSweepBufferN<MAX_RAYCAST_HITS> buffer;
        PxQueryFilterData filter;
        filter.data.word0 = mask;
        if (pxScene_->sweep(geometry, ToPxTransform(ray.origin_, rotation), ToPxVec3(ray.direction_), maxDistance, buffer, DEF_RAYCAST_FLAGS, filter))
        {
            unsigned numHits = buffer.getNbAnyHits();
            if (numHits)
            {
                results.Clear();
                results.Reserve(numHits);
                for (unsigned i = 0; i < numHits; ++i)
                {
                    const PxSweepHit& hit = buffer.getAnyHit(i);
                    PhysXRaycastResult result;
                    result.actor_ = static_cast<RigidActor*>(hit.actor->userData);
                    result.shape_ = static_cast<CollisionShape*>(hit.shape->userData);
                    result.distance_ = hit.distance;
                    result.normal_ = ToVector3(hit.normal);
                    result.position_ = ToVector3(hit.position);
                    results.Push(result);
                }
                Sort(results.Begin(), results.End(), ComparePhysXRaycastResults);
                return true;
            }
        }
    }
    return false;
}

bool Urho3DPhysX::PhysXScene::SweepSingle(PhysXRaycastResult & result, const Ray & ray, const Quaternion& rotation, const PxGeometry & geometry, float maxDistance, unsigned mask)
{
    URHO3D_PROFILE(PhysXSweepSingle);
    if (pxScene_)
    {
        PxSweepBuffer buffer;
        PxQueryFilterData filter;
        filter.data.word0 = mask;
        if (pxScene_->sweep(geometry, ToPxTransform(ray.origin_, rotation), ToPxVec3(ray.direction_), maxDistance, buffer, DEF_RAYCAST_FLAGS, filter))
        {
            if (buffer.hasBlock)
            {
                const PxSweepHit& block = buffer.block;
                result.actor_ = static_cast<RigidActor*>(block.actor->userData);
                result.shape_ = static_cast<CollisionShape*>(block.shape->userData);
                result.distance_ = block.distance;
                result.normal_ = ToVector3(block.normal);
                result.position_ = ToVector3(block.position);
                return true;
            }
        }
    }
    return false;
}

void Urho3DPhysX::PhysXScene::ReleaseScene()
{
    if (pxScene_)
    {
        for (auto* a : rigidActors_)
            a->RemoveFromScene();
        UnsubscribeFromEvent(E_SCENESUBSYSTEMUPDATE);
        
        pxScene_->release();
        pxScene_ = nullptr;
    }
}

void Urho3DPhysX::PhysXScene::HandlePostRenderUpdate(StringHash eventType, VariantMap & eventData)
{
    Scene* s = node_->GetScene();
    if (s)
    {
        DebugRenderer* dbr = s->GetOrCreateComponent<DebugRenderer>();
        DrawDebugGeometry(dbr, false);
    }
}
