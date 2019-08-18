#include "DistanceJoint.h"
#include "Physics.h"
#include "RigidActor.h"
#include "PhysXUtils.h"
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Context.h>
#include <extensions/PxDistanceJoint.h>

Urho3DPhysX::DistanceJoint::DistanceJoint(Context * context) : Joint(context),
minDistance_(0.0f),
maxDistance_(0.0f),
tolerance_(0.025f),
minDistanceEnabled_(true),
maxDistanceEnabled_(true),
springEnabled_(false)
{
}

Urho3DPhysX::DistanceJoint::~DistanceJoint()
{
}

void Urho3DPhysX::DistanceJoint::RegisterObject(Context * context)
{
    context->RegisterFactory<DistanceJoint>("PhysX");
    URHO3D_COPY_BASE_ATTRIBUTES(Joint);
    URHO3D_ACCESSOR_ATTRIBUTE("Min distance", GetMinDistance, SetMinDistance, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Min distance enabled", IsMinDistanceEnabled, SetMinDistanceEnabled, bool, true, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Max distance", GetMaxDistance, SetMaxDistance, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Max distance enabled", IsMaxDistanceEnabled, SetMaxDistanceEnabled, bool, true, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Tolerance", GetTolerance, SetTolerance, float, 0.025f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Spring enabled", IsSpringEnabled, SetSpringEnabled, bool, false, AM_DEFAULT);
}

float Urho3DPhysX::DistanceJoint::GetDistance() const
{
    if (joint_)
    {
        PxDistanceJoint* joint = joint_->is<PxDistanceJoint>();
        if (joint)
            return joint->getDistance();
    }
    //TODO: else check distance between nodes/bodies
    return 0.0f;
}

void Urho3DPhysX::DistanceJoint::SetMinDistance(float value)
{
    minDistance_ = Min(value, maxDistance_);
    if (joint_)
    {
        PxDistanceJoint* distanceJoint = joint_->is<PxDistanceJoint>();
        if (distanceJoint)
            distanceJoint->setMinDistance(minDistance_);
    }
}

void Urho3DPhysX::DistanceJoint::SetMaxDistance(float value)
{
    maxDistance_ = Max(minDistance_, value);
    if (joint_)
    {
        PxDistanceJoint* distanceJoint = joint_->is<PxDistanceJoint>();
        if (distanceJoint)
            distanceJoint->setMaxDistance(maxDistance_);
    }
}

void Urho3DPhysX::DistanceJoint::SetTolerance(float value)
{
    tolerance_ = value;
    if (joint_)
    {
        PxDistanceJoint* distanceJoint = joint_->is<PxDistanceJoint>();
        if (distanceJoint)
            distanceJoint->setTolerance(tolerance_);
    }
}

void Urho3DPhysX::DistanceJoint::SetMinDistanceEnabled(bool enable)
{
    minDistanceEnabled_ = enable;
    if (joint_)
    {
        PxDistanceJoint* distanceJoint = joint_->is<PxDistanceJoint>();
        if (distanceJoint)
            distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, minDistanceEnabled_);
    }
}

void Urho3DPhysX::DistanceJoint::SetMaxDistanceEnabled(bool enable)
{
    maxDistanceEnabled_ = enable;
    if (joint_)
    {
        PxDistanceJoint* distanceJoint = joint_->is<PxDistanceJoint>();
        if (distanceJoint)
            distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, maxDistanceEnabled_);
    }
}

void Urho3DPhysX::DistanceJoint::SetSpringEnabled(bool enable)
{
    springEnabled_ = enable;
    if (joint_)
    {
        PxDistanceJoint* distanceJoint = joint_->is<PxDistanceJoint>();
        if (distanceJoint)
            distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, springEnabled_);
    }
}

void Urho3DPhysX::DistanceJoint::CreateJointInternal()
{
    auto* physics = GetSubsystem<Physics>();
    if (!physics)
    {
        URHO3D_LOGERROR("Create joint: physics subsystem is not created.");
        return;
    }
    auto* px = physics->GetPhysics();
    PxRigidActor* ownActor = ownActor_ ? ownActor_->GetActor() : nullptr;
    PxRigidActor* otherActor = otherActor_ ? otherActor_->GetActor() : nullptr;
    PxDistanceJoint* joint = PxDistanceJointCreate(*px, ownActor, ToPxTransform(GetScaledPosition(ownActor_, position_), rotation_), otherActor, ToPxTransform(GetScaledPosition(otherActor_, otherPosition_), otherRotation_));
    joint_ = joint;
    //(re)set attributes in case of recreation of joint
    if (joint)
    {
        joint->setMinDistance(minDistance_);
        joint->setMaxDistance(maxDistance_);
        joint->setTolerance(tolerance_);
        joint->setStiffness(stiffness_);
        joint->setDamping(damping_);
        joint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, minDistanceEnabled_);
        joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, maxDistanceEnabled_);
        joint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, springEnabled_);
    }
}

void Urho3DPhysX::DistanceJoint::UpdateStiffness()
{
    if (joint_)
    {
        PxDistanceJoint* distanceJoint = joint_->is<PxDistanceJoint>();
        if (distanceJoint)
            distanceJoint->setStiffness(stiffness_);
    }
}

void Urho3DPhysX::DistanceJoint::UpdateDamping()
{
    if (joint_)
    {
        PxDistanceJoint* distanceJoint = joint_->is<PxDistanceJoint>();
        if (distanceJoint)
            distanceJoint->setDamping(damping_);
    }
}
