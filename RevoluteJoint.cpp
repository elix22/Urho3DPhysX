#include "RevoluteJoint.h"
#include "RigidActor.h"
#include "Physics.h"
#include "PhysXUtils.h"
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Context.h>
#include <extensions/PxRevoluteJoint.h>

Urho3DPhysX::RevoluteJoint::RevoluteJoint(Context * context) : Joint(context),
driveVelocity_(0.0f),
driveForceLimit_(PX_MAX_F32),
driveGearRatio_(1.0f),
lowerLimit_(0.0f),
upperLimit_(0.0f),
contactDistance_(-1.0f),
limitEnabled_(false),
driveEnabled_(false),
driveFreeSpin_(false)
{
}

Urho3DPhysX::RevoluteJoint::~RevoluteJoint()
{
}

void Urho3DPhysX::RevoluteJoint::RegisterObject(Context * context)
{
    context->RegisterFactory<RevoluteJoint>("PhysX");
    URHO3D_COPY_BASE_ATTRIBUTES(Joint);
    URHO3D_ACCESSOR_ATTRIBUTE("Drive velocity", GetDriveVelocity, SetDriveVelocity, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Drive force limit", GetDriveForceLimit, SetDriveForceLimit, float, PX_MAX_F32, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Drive gear ratio", GetDriveGearRatio, SetDriveGearRatio, float, 1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Lower limit", GetLowerLimit, SetLowerLimit, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Upper limit", GetUpperLimit, SetUpperLimit, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Contact distance", GetContactDistance, SetContactDistance, float, -1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Limit enabled", IsLimitEnabled, SetLimitEnabled, bool, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Drive enabled", IsDriveEnabled, SetDriveEnabled, bool, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Drive free spin", GetDriveFreeSpin, SetDriveFreeSpin, bool, false, AM_DEFAULT);
}

void Urho3DPhysX::RevoluteJoint::SetDriveVelocity(float value)
{
    if (driveVelocity_ != value)
    {
        driveVelocity_ = value;
        if (joint_)
        {
            PxRevoluteJoint* revoluteJoint = joint_->is<PxRevoluteJoint>();
            if (revoluteJoint)
                revoluteJoint->setDriveVelocity(driveVelocity_);
        }
    }
}

void Urho3DPhysX::RevoluteJoint::SetDriveForceLimit(float value)
{
    if (driveForceLimit_ != value)
    {
        driveForceLimit_ = value;
        if (joint_)
        {
            PxRevoluteJoint* revoluteJoint = joint_->is<PxRevoluteJoint>();
            if (revoluteJoint)
                revoluteJoint->setDriveForceLimit(driveForceLimit_);
        }
    }
}

void Urho3DPhysX::RevoluteJoint::SetDriveGearRatio(float value)
{
    if (driveGearRatio_ != value)
    {
        driveGearRatio_ = value;
        if (joint_)
        {
            PxRevoluteJoint* revoluteJoint = joint_->is<PxRevoluteJoint>();
            if (revoluteJoint)
                revoluteJoint->setDriveGearRatio(driveGearRatio_);
        }
    }
}

void Urho3DPhysX::RevoluteJoint::SetUpperLimit(float value)
{
    if (upperLimit_ != value)
    {
        upperLimit_ = value;
        if (IsUsingSoftLimit())
            UpdateLimit();
    }
}

void Urho3DPhysX::RevoluteJoint::SetLowerLimit(float value)
{
    if (lowerLimit_ != value)
    {
        lowerLimit_ = value;
        if (IsUsingSoftLimit())
            UpdateLimit();
    }
}

void Urho3DPhysX::RevoluteJoint::SetContactDistance(float value)
{
    if (contactDistance_ != value)
    {
        contactDistance_ = value;
        if (!IsUsingSoftLimit())
            UpdateLimit();
    }
}

void Urho3DPhysX::RevoluteJoint::SetLimitEnabled(bool enable)
{
    if (limitEnabled_ != enable)
    {
        limitEnabled_ = enable;
        UpdateLimit();
    }
}

void Urho3DPhysX::RevoluteJoint::SetDriveEnabled(bool enable)
{
    if (driveEnabled_ != enable)
    {
        driveEnabled_ = enable;
        if (joint_)
        {
            PxRevoluteJoint* revoluteJoint = joint_->is<PxRevoluteJoint>();
            if (revoluteJoint)
            {
                revoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, driveEnabled_);
            }
        }
    }
}

void Urho3DPhysX::RevoluteJoint::SetDriveFreeSpin(bool enable)
{
    if (driveFreeSpin_ != enable)
    {
        driveFreeSpin_ = enable;
        if (joint_)
        {
            PxRevoluteJoint* revoluteJoint = joint_->is<PxRevoluteJoint>();
            if (revoluteJoint)
            {
                revoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_FREESPIN, driveFreeSpin_);
            }
        }
    }
}

void Urho3DPhysX::RevoluteJoint::CreateJointInternal()
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
    PxRevoluteJoint* joint = PxRevoluteJointCreate(*px, ownActor, ToPxTransform(GetScaledPosition(ownActor_, position_), rotation_), otherActor, ToPxTransform(GetScaledPosition(otherActor_, otherPosition_), otherRotation_));
    joint_ = joint;
    //(re)set attributes in case of recreation of joint
    if (joint)
    {
        joint->setDriveVelocity(driveVelocity_);
        joint->setDriveForceLimit(driveForceLimit_);
        joint->setDriveGearRatio(driveGearRatio_);
        joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, driveEnabled_);
        joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_FREESPIN, driveFreeSpin_);
        UpdateLimit();
    }
}

void Urho3DPhysX::RevoluteJoint::UpdateLimit()
{
    if (!limitEnabled_)
        return;
    if (joint_)
    {
        PxRevoluteJoint* revoluteJoint = joint_->is<PxRevoluteJoint>();
        if (revoluteJoint)
        {
            if (IsUsingSoftLimit())
                revoluteJoint->setLimit(PxJointAngularLimitPair(lowerLimit_, upperLimit_, PxSpring(stiffness_, damping_)));
            else
                revoluteJoint->setLimit(PxJointAngularLimitPair(lowerLimit_, upperLimit_, contactDistance_));
            revoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
        }
    }
}
