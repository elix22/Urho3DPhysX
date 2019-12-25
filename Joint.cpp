#include "Joint.h"
#include "RigidActor.h"
#include "RigidBody.h"
#include "Physics.h"
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/Profiler.h>
#include <Urho3D/IO/Log.h>

static const float DEF_BREAK_FORCE = PX_MAX_F32;

Urho3DPhysX::Joint::Joint(Context * context) : Component(context),
joint_(nullptr),
position_(Vector3::ZERO),
rotation_(Quaternion::IDENTITY),
otherPosition_(Vector3::ZERO),
otherRotation_(Quaternion::IDENTITY),
ownActor_(nullptr),
otherActor_(nullptr),
otherNodeID_(0),
needCreation_(true),
breakForce_(DEF_BREAK_FORCE),
breakTorque_(DEF_BREAK_FORCE),
stiffness_(0.0f),
damping_(0.0f),
useSoftLimit_(true),
enableCollision_(true),
otherActorNodeID_(0)
{
}

Urho3DPhysX::Joint::~Joint()
{
    ReleaseJoint(true);
}

void Urho3DPhysX::Joint::RegisterObject(Context * context)
{
    context->RegisterFactory<Joint>();
    URHO3D_ACCESSOR_ATTRIBUTE("Position", GetPosition, SetPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Rotation", GetRotation, SetRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Other position", GetOtherPosition, SetOtherPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Other rotation", GetOtherRotation, SetOtherRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Break force", GetBreakForce, SetBreakForce, float, DEF_BREAK_FORCE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Break torque", GetBreakTorque, SetBreakTorque, float, DEF_BREAK_FORCE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Stiffness", GetStiffness, SetStiffness, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Damping", GetDamping, SetDamping, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Use soft limit", IsUsingSoftLimit, SetUseSoftLimit, bool, true, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Self collision", GetCollisionEnabled, SetEnableCollision, bool, true, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Other actor", GetOtherActorNodeIDAttr, SetOtherActorNodeIDAttr, unsigned, 0, AM_DEFAULT | AM_NODEID);
}

void Urho3DPhysX::Joint::ApplyAttributes()
{
    if (needCreation_)
    {
        Scene* scene = GetScene();
        if (scene)
        {
            if (otherActorNodeID_)
            {
                Node* otherNode = scene->GetNode(otherActorNodeID_);
                if (otherNode)
                    SetOtherActor(otherNode->GetDerivedComponent<RigidActor>());
            }
            else
            {
                SetOtherActor(nullptr);
            }
        }
    }
}

void Urho3DPhysX::Joint::SetOtherActor(RigidActor * actor)
{
    if (actor && actor != otherActor_)
    {
        if (otherActor_)
        {
            otherActor_->RemoveJoint(this);
            otherActor_ = nullptr;
            otherActorNodeID_ = 0;
        }
        otherActor_ = actor;
        otherActorNodeID_ = otherActor_->GetNode()->GetID();
        otherActor_->AddJoint(this);
        CreateJoint();
    }
    else if (!actor && otherActor_)
    {
        otherActor_->RemoveJoint(this);
        otherActor_ = nullptr;
        otherActorNodeID_ = 0;
        CreateJoint();
    }
}

void Urho3DPhysX::Joint::CreateJoint()
{
    URHO3D_PROFILE(CreateJoint);
    
    cachedWorldScale_ = node_->GetWorldScale();

    ReleaseJoint(false);
    ownActor_ = node_->GetDerivedComponent<RigidActor>();
    PxRigidActor* ownActor = ownActor_ ? ownActor_->GetActor() : nullptr;

    if (!ownActor)
    {
        needCreation_ = true;
        return;
    }
    CreateJointInternal();
    if (joint_)
    {
        //set joint paramters
        joint_->setBreakForce(breakForce_, breakTorque_);
        joint_->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, enableCollision_);
#ifdef _DEBUG
        joint_->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
#endif // _DEBUG
        joint_->userData = this;
        ownActor_->AddJoint(this);
    }
    needCreation_ = false;
}

void Urho3DPhysX::Joint::SetTransform(const Vector3 & position, const Quaternion & rotation)
{
    bool needUpdate = false;
    if (position_ != position)
    {
        position_ = position;
        needUpdate = true;
    }
    if (rotation_ != rotation)
    {
        rotation_ = rotation;
        needUpdate = true;
    }
    if (needUpdate && joint_)
        joint_->setLocalPose(PxJointActorIndex::eACTOR0, ToPxTransform(GetScaledPosition(ownActor_, position_), rotation_));
}

void Urho3DPhysX::Joint::SetPosition(const Vector3 & position)
{
    SetTransform(position, rotation_);
}

void Urho3DPhysX::Joint::SetRotation(const Quaternion & rotation)
{
    SetTransform(position_, rotation);
}

void Urho3DPhysX::Joint::SetOtherTransform(const Vector3 & position, const Quaternion & rotation)
{
    bool needUpdate = false;
    if (otherPosition_ != position)
    {
        otherPosition_ = position;
        needUpdate = true;
    }
    if (otherRotation_ != rotation)
    {
        otherRotation_ = rotation;
        needUpdate = true;
    }
    if (needUpdate && joint_)
        joint_->setLocalPose(PxJointActorIndex::eACTOR1, ToPxTransform(GetScaledPosition(otherActor_, otherPosition_), otherRotation_));
}

void Urho3DPhysX::Joint::SetOtherPosition(const Vector3 & position)
{
    SetOtherTransform(position, otherRotation_);
}

void Urho3DPhysX::Joint::SetOtherRotation(const Quaternion & rotation)
{
    SetOtherTransform(otherPosition_, rotation);
}

void Urho3DPhysX::Joint::SetBreakForce(float force)
{
    if (breakForce_ != force)
    {
        breakForce_ = force;
        if (joint_)
            joint_->setBreakForce(breakForce_, breakTorque_);
    }
}

void Urho3DPhysX::Joint::SetBreakTorque(float torque)
{
    if (breakTorque_ != torque)
    {
        breakTorque_ = torque;
        if (joint_)
            joint_->setBreakForce(breakForce_, breakTorque_);
    }
}

void Urho3DPhysX::Joint::SetStiffness(float value)
{
    if (stiffness_ != value)
    {
        stiffness_ = value;
        UpdateStiffness();
    }
}

void Urho3DPhysX::Joint::SetDamping(float value)
{
    if (damping_ != value)
    {
        damping_ = value;
        UpdateDamping();
    }
}

void Urho3DPhysX::Joint::SetUseSoftLimit(bool use)
{
    if (useSoftLimit_ != use)
    {
        useSoftLimit_ = use;
        UpdateUseSoftLimit();
    }
}

void Urho3DPhysX::Joint::SetEnableCollision(bool value)
{
    if(enableCollision_ != value)
    {
        enableCollision_ = value;
        if (joint_)
        {
            joint_->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, enableCollision_);
        }
    }
}

void Urho3DPhysX::Joint::SetOtherActorNodeIDAttr(unsigned value)
{
    if (otherActorNodeID_ != value)
    {
        otherActorNodeID_ = value;
        needCreation_ = true;
    }
}

void Urho3DPhysX::Joint::ReleaseJoint(bool removeFromActors)
{
    if (removeFromActors)
    {
        for (auto a : { ownActor_, otherActor_ })
        {
            if (a)
                a->RemoveJoint(this);
        }
    }
    if (joint_)
        joint_->release();
}

/*void Urho3DPhysX::Joint::OnNodeSet(Node * node)
{
    node->AddListener(this);
}

void Urho3DPhysX::Joint::OnMarkedDirty(Node * node)
{
    //TODO wake up actors if necessery
}*/

Vector3 Urho3DPhysX::Joint::GetScaledPosition(RigidActor * actor, const Vector3 & position)
{
    if (actor)
    {
        RigidBody* rb = actor->Cast<RigidBody>();
        Vector3 centerOfMass = rb ? rb->GetCenterOfMass() : Vector3::ZERO;
        return position * actor->GetNode()->GetWorldScale() - centerOfMass;
    }
    return Vector3();
}
