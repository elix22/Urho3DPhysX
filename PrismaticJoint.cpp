#include "PrismaticJoint.h"
#include "Physics.h"
#include "RigidActor.h"
#include "PhysXUtils.h"
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Context.h>
#include <extensions/PxPrismaticJoint.h>

Urho3DPhysX::PrismaticJoint::PrismaticJoint(Context * context) : Joint(context),
lowerLimit_(-1.0f),
upperLimit_(1.0f),
contactDistance_(-1.0f),
limitEnabled_(false)
{
}

Urho3DPhysX::PrismaticJoint::~PrismaticJoint()
{
}

void Urho3DPhysX::PrismaticJoint::RegisterObject(Context * context)
{
    context->RegisterFactory<PrismaticJoint>("PhysX");
    URHO3D_COPY_BASE_ATTRIBUTES(Joint);
    URHO3D_ACCESSOR_ATTRIBUTE("Upper limit", GetUpperLimit, SetUpperLimit, float, -1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Lower Limit", GetLowerLimit, SetLowerLimit, float, 1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Contact distance", GetContactDistance, SetContactDistance, float, -1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Limit enabled", IsLimitEnabled, SetLimitEnabled, bool, false, AM_DEFAULT);
}

void Urho3DPhysX::PrismaticJoint::SetUpperLimit(float value)
{
    if (upperLimit_ != value)
    {
        upperLimit_ = Max(value, lowerLimit_);
        UpdateLimit();
    }
}

void Urho3DPhysX::PrismaticJoint::SetLowerLimit(float value)
{
    if (lowerLimit_ != value)
    {
        lowerLimit_ = Min(value, upperLimit_);
        UpdateLimit();
    }
}

void Urho3DPhysX::PrismaticJoint::SetContactDistance(float value)
{
    if (contactDistance_ != value)
    {
        contactDistance_ = value;
        UpdateLimit();
    }
}

void Urho3DPhysX::PrismaticJoint::SetLimitEnabled(bool enable)
{
    if (limitEnabled_ != enable)
    {
        limitEnabled_ = enable;
        UpdateLimit();
    }
}

void Urho3DPhysX::PrismaticJoint::CreateJointInternal()
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
    PxPrismaticJoint* joint = PxPrismaticJointCreate(*px, ownActor, ToPxTransform(GetScaledPosition(ownActor_, position_), rotation_), otherActor, ToPxTransform(GetScaledPosition(otherActor_, otherPosition_), otherRotation_));
    joint_ = joint;
    //(re)set attributes in case of recreation of joint
    if (joint_)
    {
        UpdateLimit();
    }
}

void Urho3DPhysX::PrismaticJoint::UpdateLimit()
{
    if (joint_ && limitEnabled_)
    {
        PxPrismaticJoint* prismaticJoint = joint_->is<PxPrismaticJoint>();
        if (prismaticJoint)
        {
            prismaticJoint->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
            if (IsUsingSoftLimit())
                prismaticJoint->setLimit(PxJointLinearLimitPair(lowerLimit_, upperLimit_, PxSpring(stiffness_, damping_)));
            else
            {
                auto* px = GetSubsystem<Physics>()->GetPhysics();
                prismaticJoint->setLimit(PxJointLinearLimitPair(px->getTolerancesScale(), lowerLimit_, upperLimit_, contactDistance_));
            }
        }
    }
}

void Urho3DPhysX::PrismaticJoint::UpdateStiffness()
{
    if (IsUsingSoftLimit())
        UpdateLimit();
}

void Urho3DPhysX::PrismaticJoint::UpdateDamping()
{
    if (IsUsingSoftLimit())
        UpdateLimit();
}
