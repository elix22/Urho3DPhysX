#include "FixedJoint.h"
#include "RigidActor.h"
#include "Physics.h"
#include "PhysXUtils.h"
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Context.h>
#include <extensions/PxFixedJoint.h>

Urho3DPhysX::FixedJoint::FixedJoint(Context * context) : Joint(context),
projectionLinearTolerance_(0.0f),
projectionAngularTolerance_(0.0f)
{
}

Urho3DPhysX::FixedJoint::~FixedJoint()
{
}

void Urho3DPhysX::FixedJoint::RegisterObject(Context * context)
{
    context->RegisterFactory<FixedJoint>("PhysX");
    URHO3D_COPY_BASE_ATTRIBUTES(Joint);
    URHO3D_ACCESSOR_ATTRIBUTE("PLT", GetProjectionLinearTolerance, SetProjectionLinearTolerance, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("PAT", GetProjectionAngularTolerance, SetProjectionAngularTolerance, float, 0.0f, AM_DEFAULT);
}

void Urho3DPhysX::FixedJoint::SetProjectionLinearTolerance(float value)
{
    projectionLinearTolerance_ = value;
    if (joint_)
    {
        PxFixedJoint* joint = joint_->is<PxFixedJoint>();
        if (joint)
            joint->setProjectionLinearTolerance(projectionLinearTolerance_);
    }
}

void Urho3DPhysX::FixedJoint::SetProjectionAngularTolerance(float value)
{
    projectionAngularTolerance_ = value;
    if (joint_)
    {
        PxFixedJoint* joint = joint_->is<PxFixedJoint>();
        if (joint)
            joint->setProjectionAngularTolerance(projectionAngularTolerance_);
    }
}

void Urho3DPhysX::FixedJoint::CreateJointInternal()
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
    PxFixedJoint* joint = PxFixedJointCreate(*px, ownActor, ToPxTransform(GetScaledPosition(ownActor_, position_), rotation_), otherActor, ToPxTransform(GetScaledPosition(otherActor_, otherPosition_), otherRotation_));
    joint_ = joint;
    //(re)set attributes in case of recreation of joint
    if (joint)
    {
        joint->setProjectionLinearTolerance(projectionLinearTolerance_);
        joint->setProjectionAngularTolerance(projectionAngularTolerance_);
    }
}
