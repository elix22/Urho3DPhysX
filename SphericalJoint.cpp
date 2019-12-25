#include "SphericalJoint.h"
#include "RigidActor.h"
#include "Physics.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/IO/Log.h>
#include <extensions/PxSphericalJoint.h>

namespace Urho3DPhysX
{
    static const float MIN_LIMIT_CONE_AXIS = 0.001f;
    static const Vector2 DEF_LIMITCONE = Vector2(PxPi / 2, PxPi / 2);
}

Urho3DPhysX::SphericalJoint::SphericalJoint(Context * context) : Joint(context),
limitConeAngles_(DEF_LIMITCONE),
contactDistance_(-1.0f),
limitEnabled_(false)
{
}

Urho3DPhysX::SphericalJoint::~SphericalJoint()
{
}

void Urho3DPhysX::SphericalJoint::RegisterObject(Context * context)
{
    context->RegisterFactory<SphericalJoint>("PhysX");
    URHO3D_COPY_BASE_ATTRIBUTES(Joint);
    URHO3D_ACCESSOR_ATTRIBUTE("Limit cone", GetLimitConeAngles, SetLimitConeAngles, Vector2, DEF_LIMITCONE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Contact distance", GetContactDistance, SetContactDistance, float, -1.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Limit enabled", IsLimitEnabled, SetLimitEnabled, bool, false, AM_DEFAULT);
}

void Urho3DPhysX::SphericalJoint::SetLimitConeAngles(const Vector2 & axesLimit)
{
    Vector2 clampedValue;
    clampedValue.x_ = Clamp(axesLimit.x_, MIN_LIMIT_CONE_AXIS, PxPi);
    clampedValue.y_ = Clamp(axesLimit.y_, MIN_LIMIT_CONE_AXIS, PxPi);
    if (limitConeAngles_ != clampedValue)
    {
        limitConeAngles_ = clampedValue;
        ApplyLimitCone();
    }
}

void Urho3DPhysX::SphericalJoint::SetContactDistance(float value)
{
    if (contactDistance_ != value)
    {
        contactDistance_ = value;
        if (!IsUsingSoftLimit())
            ApplyLimitCone();
    }
}

void Urho3DPhysX::SphericalJoint::SetLimitEnabled(bool value)
{
    if (limitEnabled_ != value)
    {
        limitEnabled_ = value;
        ApplyLimitCone();
    }
}

void Urho3DPhysX::SphericalJoint::CreateJointInternal()
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
    joint_ = PxSphericalJointCreate(*px, ownActor, ToPxTransform(GetScaledPosition(ownActor_, position_), rotation_), otherActor, ToPxTransform(GetScaledPosition(otherActor_, otherPosition_), otherRotation_));
    //(re)set attributes in case of recreation of joint
    if (joint_)
    {
        ApplyLimitCone();
    }
}

void Urho3DPhysX::SphericalJoint::UpdateStiffness()
{
    if (IsUsingSoftLimit())
        ApplyLimitCone();
}

void Urho3DPhysX::SphericalJoint::UpdateDamping()
{
    if (IsUsingSoftLimit())
        ApplyLimitCone();
}

void Urho3DPhysX::SphericalJoint::ApplyLimitCone()
{
    if (!limitEnabled_)
        return;
    if (joint_)
    {
        PxSphericalJoint* sphericalJoint = joint_->is<PxSphericalJoint>();
        if (sphericalJoint)
        {
            if (IsUsingSoftLimit())
                sphericalJoint->setLimitCone(PxJointLimitCone(limitConeAngles_.x_, limitConeAngles_.y_, PxSpring(stiffness_, damping_)));
            else
                sphericalJoint->setLimitCone(PxJointLimitCone(limitConeAngles_.x_, limitConeAngles_.y_, contactDistance_));
            sphericalJoint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
        }
    }
}
