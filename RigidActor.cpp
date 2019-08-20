#include "RigidActor.h"
#include "PhysXScene.h"
#include "PhysXUtils.h"
#include "CollisionShape.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/DebugRenderer.h>

Urho3DPhysX::RigidActor::RigidActor(Context * context) : Component(context),
isApplyingTransform_(false),
pxScene_(nullptr)
{
}

Urho3DPhysX::RigidActor::~RigidActor()
{
    ReleaseActor();
}

void Urho3DPhysX::RigidActor::RegisterObject(Context * context)
{
    context->RegisterFactory<RigidActor>();
}

void Urho3DPhysX::RigidActor::DrawDebugGeometry(DebugRenderer * debug, bool depthTest)
{
    if (debug && actor_)
    {
        debug->AddSphere(Sphere(ToVector3(actor_->getGlobalPose().p), 0.10f), Color::GREEN, depthTest);
    }
}

void Urho3DPhysX::RigidActor::OnSetEnabled()
{
    if (actor_)
    {
        if (IsEnabledEffective())
        {
            actor_->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
        }
        else
        {
            actor_->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, true);
        }
    }
}

bool Urho3DPhysX::RigidActor::AttachShape(CollisionShape * shape, bool updateMassAndInteria)
{
    if (shape && actor_)
    {
        if (actor_->attachShape(*shape->GetShape()))
        {
            shape->SetActor(this);
            return true;
        }
    }
    return false;
}

void Urho3DPhysX::RigidActor::ApplyWorldTransformFromActor()
{
    if (actor_)
    {
        const PxTransform& pose = actor_->getGlobalPose();
        ApplyWorldTransform(pose.p, pose.q);
    }
}

void Urho3DPhysX::RigidActor::ApplyWorldTransform(const Vector3 & worldPosition, const Quaternion & worldRotation)
{
    if (node_)
    {
        isApplyingTransform_ = true;
        node_->SetWorldPosition(worldPosition);
        node_->SetWorldRotation(worldRotation);
        isApplyingTransform_ = false;
    }
}

void Urho3DPhysX::RigidActor::ApplyWorldTransform(const PxVec3 & worldPosition, const PxQuat & worldRotation)
{
    ApplyWorldTransform(ToVector3(worldPosition), ToQuaternion(worldRotation));
}

void Urho3DPhysX::RigidActor::UpdateTransformFromNode(bool awake)
{
    if (node_)
    {
        actor_->setGlobalPose(ToPxTransform(node_->GetWorldTransform()), awake);
    }
}

void Urho3DPhysX::RigidActor::SetTransform(const Vector3 & position, const Quaternion & rotation, bool awake)
{
    actor_->setGlobalPose(ToPxTransform(position, rotation), awake);
}

void Urho3DPhysX::RigidActor::SetTransform(const Matrix3x4 & matrix, bool awake)
{
    actor_->setGlobalPose(ToPxTransform(matrix), awake);
}

void Urho3DPhysX::RigidActor::AddJoint(Joint * joint)
{
    joints_.Push(joint);
    OnJointAdded(joint);
}

void Urho3DPhysX::RigidActor::RemoveJoint(Joint * joint)
{
    if (joints_.Remove(joint))
    {
        OnJointRemoved(joint);
    }
}

void Urho3DPhysX::RigidActor::RemoveFromScene()
{
    if (pxScene_)
        pxScene_->RemoveActor(this);
    pxScene_.Reset();
}

void Urho3DPhysX::RigidActor::ReleaseActor()
{
    if (actor_)
    {
        PxShape* shapes;
        actor_->getShapes(&shapes, actor_->getNbShapes());
        for (unsigned i = 0; i < actor_->getNbShapes(); ++i)
        {
            actor_->detachShape(shapes[i]);
        }
        if(pxScene_)
            pxScene_->RemoveActor(this);
        pxScene_ = nullptr;
        actor_->userData = nullptr;
        actor_->release();
        actor_ = nullptr;
    }
}

void Urho3DPhysX::RigidActor::OnSceneSet(Scene * scene)
{
    if (scene)
    {
        if (scene == node_)
            URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

        pxScene_ = scene->GetOrCreateComponent<PhysXScene>();
        pxScene_->AddActor(this);
    }
}

void Urho3DPhysX::RigidActor::OnNodeSet(Node * node)
{
    if (node)
    {
        node->AddListener(this);
        SetTransform(node->GetWorldTransform());
        //check if collision shapes already exist and if yes, attach them.
        PODVector<CollisionShape*> shapes;
        node->GetComponents<CollisionShape>(shapes);
        for (auto* shape : shapes)
            AttachShape(shape, false);
    }
}

void Urho3DPhysX::RigidActor::OnMarkedDirty(Node * node)
{
    if (isApplyingTransform_)
        return;
    UpdateTransformFromNode();
}
