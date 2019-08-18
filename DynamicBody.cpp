#include "DynamicBody.h"
#include "Physics.h"
#include "PhysXUtils.h"
#include "Joint.h"
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Context.h>

Urho3DPhysX::DynamicBody::DynamicBody(Context * context) : RigidBody(context)
{
    auto* physics = GetSubsystem<Physics>();
    if (physics)
    {
        auto* px = physics->GetPhysics();
        if (px)
        {
            actor_= physics->GetPhysics()->createRigidDynamic(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
            actor_->userData = this;
        }
    }
    else
    {
        URHO3D_LOGERROR("Physics subsystem must be registred before creating physx objects.");
    }
}

Urho3DPhysX::DynamicBody::~DynamicBody()
{
}

void Urho3DPhysX::DynamicBody::RegisterObject(Context * context)
{
    context->RegisterFactory<DynamicBody>("PhysX");
    URHO3D_COPY_BASE_ATTRIBUTES(RigidBody);
    URHO3D_ACCESSOR_ATTRIBUTE("Sleep threshold", GetSleepThreshold, SetSleepThreshold, float, 0.01f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Stabilization threshold", GetStabilizationThreshold, SetStabilizationThreshold, float, 0.0025f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Contact report threshold", GetContactReportThreshold, SetContactReportTheshold, float, PX_MAX_F32, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Lock linear", GetLockLinear, SetLockLinear, IntVector3, IntVector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Lock angular", GetLockAngular, SetLockAngular, IntVector3, IntVector3::ZERO, AM_DEFAULT);
}

void Urho3DPhysX::DynamicBody::OnSetEnabled()
{
    RigidActor::OnSetEnabled();
    if (IsEnabledEffective())
        WakeUp();
}

void Urho3DPhysX::DynamicBody::ApplyForce(const Vector3 & force)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->addForce(ToPxVec3(force), PxForceMode::eFORCE, true);
}

void Urho3DPhysX::DynamicBody::ApplyImpulse(const Vector3 & force)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->addForce(ToPxVec3(force), PxForceMode::eIMPULSE, true);
}

void Urho3DPhysX::DynamicBody::ApplyVelocityChange(const Vector3 & force)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->addForce(ToPxVec3(force), PxForceMode::eVELOCITY_CHANGE, true);
}

void Urho3DPhysX::DynamicBody::ApplyAcceleration(const Vector3 & force)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->addForce(ToPxVec3(force), PxForceMode::eACCELERATION, true);
}

void Urho3DPhysX::DynamicBody::ApplyTorque(const Vector3 & torque)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->addTorque(ToPxVec3(torque), PxForceMode::eFORCE, true);
}

void Urho3DPhysX::DynamicBody::ApplyTorqueImpulse(const Vector3 & torque)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->addTorque(ToPxVec3(torque), PxForceMode::eIMPULSE, true);
}

void Urho3DPhysX::DynamicBody::ApplyTorqueVelocityChange(const Vector3 & torque)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->addTorque(ToPxVec3(torque), PxForceMode::eVELOCITY_CHANGE, true);
}

void Urho3DPhysX::DynamicBody::ApplyTorqueAcceleration(const Vector3 & torque)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->addTorque(ToPxVec3(torque), PxForceMode::eACCELERATION, true);
}

void Urho3DPhysX::DynamicBody::SetSleepThreshold(float value)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setSleepThreshold(value);
}

float Urho3DPhysX::DynamicBody::GetSleepThreshold() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? rigidDynamic->getSleepThreshold() : 0.0f;
}

bool Urho3DPhysX::DynamicBody::IsSleeping() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? rigidDynamic->isSleeping() : true;
}

void Urho3DPhysX::DynamicBody::WakeUp()
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if(rigidDynamic)
        rigidDynamic->wakeUp();
}

void Urho3DPhysX::DynamicBody::PutToSleep()
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->putToSleep();
}

void Urho3DPhysX::DynamicBody::SetStabilizationThreshold(float value)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setStabilizationThreshold(value);
}

float Urho3DPhysX::DynamicBody::GetStabilizationThreshold() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? rigidDynamic->getStabilizationThreshold() : 0.0f;
}

void Urho3DPhysX::DynamicBody::SetContactReportTheshold(float value)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setContactReportThreshold(value);
}

float Urho3DPhysX::DynamicBody::GetContactReportThreshold() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? rigidDynamic->getContactReportThreshold() : 0.0f;
}

void Urho3DPhysX::DynamicBody::SetLockLinearX(bool lock)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_X, lock);
}

bool Urho3DPhysX::DynamicBody::GetLockLinearX() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? (rigidDynamic->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_LINEAR_X) : false;
}

void Urho3DPhysX::DynamicBody::SetLockLinearY(bool lock)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y, lock);
}

bool Urho3DPhysX::DynamicBody::GetLockLinearY() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? (rigidDynamic->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y) : false;
}

void Urho3DPhysX::DynamicBody::SetLockLinearZ(bool lock)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Z, lock);
}

bool Urho3DPhysX::DynamicBody::GetLockLinearZ() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? (rigidDynamic->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z) : false;
}

void Urho3DPhysX::DynamicBody::SetLockLinear(const IntVector3 & lock)
{
    SetLockLinearX(lock.x_);
    SetLockLinearY(lock.y_);
    SetLockLinearZ(lock.z_);
}

IntVector3 Urho3DPhysX::DynamicBody::GetLockLinear() const
{
    return IntVector3(GetLockLinearX(), GetLockLinearY(), GetLockLinearZ());
}

void Urho3DPhysX::DynamicBody::SetLockAngularX(bool lock)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_X, lock);
}

bool Urho3DPhysX::DynamicBody::GetLockAngularX() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? (rigidDynamic->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X) : false;
}

void Urho3DPhysX::DynamicBody::SetLockAngularY(bool lock)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y, lock);
}

bool Urho3DPhysX::DynamicBody::GetLockAngularY() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? (rigidDynamic->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y) : false;
}

void Urho3DPhysX::DynamicBody::SetLockAngularZ(bool lock)
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, lock);
}

bool Urho3DPhysX::DynamicBody::GetLockAngularZ() const
{
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    return rigidDynamic ? (rigidDynamic->getRigidDynamicLockFlags() & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z) : false;
}

void Urho3DPhysX::DynamicBody::SetLockAngular(const IntVector3 & lock)
{
    SetLockAngularX(lock.x_);
    SetLockAngularY(lock.y_);
    SetLockAngularZ(lock.z_);
}

IntVector3 Urho3DPhysX::DynamicBody::GetLockAngular() const
{
    return IntVector3(GetLockAngularX(), GetLockAngularY(), GetLockAngularZ());
}

void Urho3DPhysX::DynamicBody::SetKinematicTarget(const Vector3 & position, const Quaternion & rotation)
{
    if (!kinematic_ || !IsEnabled())
        return;
    PxRigidDynamic* rigidDynamic = actor_->is<PxRigidDynamic>();
    if (rigidDynamic)
        rigidDynamic->setKinematicTarget(ToPxTransform(position, rotation));
}

bool Urho3DPhysX::DynamicBody::GetKinematicTarget(Vector3 & position, Quaternion & rotation)
{
    PxRigidDynamic* dynamicBody = actor_->is<PxRigidDynamic>();
    if (dynamicBody)
    {
        PxTransform transform;
        if (dynamicBody->getKinematicTarget(transform))
        {
            position = ToVector3(transform.p);
            rotation = ToQuaternion(transform.q);
            return true;
        }
    }
    return false;
}

void Urho3DPhysX::DynamicBody::OnMarkedDirty(Node * node)
{
    RigidBody::OnMarkedDirty(node);
    if(IsSleeping())
        WakeUp();
}

void Urho3DPhysX::DynamicBody::OnJointAdded(Joint * joint)
{
    if (joint)
    {
        WakeUp();
        //joint->GetNode()->AddListener(this);
    }
}

void Urho3DPhysX::DynamicBody::OnJointRemoved(Joint * joint)
{
    if (joint)
    {
        WakeUp();
        //joint->GetNode()->RemoveListener(this);
    }
}
