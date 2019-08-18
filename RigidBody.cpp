#include "RigidBody.h"
#include "PhysXUtils.h"
#include "CollisionShape.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/IO/Log.h>
#include <PxRigidBody.h>
#include <extensions/PxRigidBodyExt.h>

namespace Urho3DPhysX
{
    static const float DEF_MASS = 1.0f;
}

Urho3DPhysX::RigidBody::RigidBody(Context * context) : RigidActor(context),
mass_(DEF_MASS),
centerOfMass_(Vector3::ZERO),
isDirty_(true),
ccdEnabled_(false),
kinematic_(false)
{
}

Urho3DPhysX::RigidBody::~RigidBody()
{
}

void Urho3DPhysX::RigidBody::RegisterObject(Context * context)
{
    context->RegisterFactory<RigidBody>();
    URHO3D_COPY_BASE_ATTRIBUTES(RigidActor);
    URHO3D_ACCESSOR_ATTRIBUTE("Mass", GetMass, SetMass, float, DEF_MASS, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Center of mass", GetCenterOfMass, SetCenterOfMass, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("CCD", IsCCDEnabled, SetCCDEnabled, bool, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Kinematic", IsKinematic, SetKinematic, bool, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Linear Damping", GetLinearDamping, SetLinearDamping, float, 0.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular Damping", GetAngularDamping, SetAngularDamping, float, 0.05f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Max linear velocity", GetMaxLinearVelocity, SetMaxLinearVelocity, float, PX_MAX_F32, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Linear velocity", GetLinearVelocity, SetLinearVelocity, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Max angular velocity", GetMaxAngularVelocity, SetMaxAngularVelocity, float, 100.0f, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Angular velocity", GetAngularVelocity, SetAngularVelocity, Vector3, Vector3::ZERO, AM_DEFAULT);
}

bool Urho3DPhysX::RigidBody::AttachShape(CollisionShape * shape, bool updateMassAndInteria)
{
    if (shape)
    {
        ///check if shape doesn't require kinematic flag
        PhysXShapeType type = shape->GetShapeType();
        if (type == TRIANGLEMESH_SHAPE || type == PLANE_SHAPE /* || heightfield*/)
        {
            PxRigidBody* body = actor_->is<PxRigidBody>();
            if (body && !(body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
            {
                URHO3D_LOGWARNING("Attempt to set triangle mesh/plane/heightfield collision shape on non-kinematic body. Exluding collision shape from simulation.");
                shape->GetShape()->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);  //alterative: set kinematic
            }
        }
    }
    if (RigidActor::AttachShape(shape))
    {
        if(updateMassAndInteria)
            UpdateMassAndInertia();
        return true;
    }
    return false;
}

void Urho3DPhysX::RigidBody::ApplyAttributes()
{
    if (isDirty_)
        UpdateMassAndInertia();
}

void Urho3DPhysX::RigidBody::SetMass(float mass)
{
    mass = Max(mass, 0.0f);
    if (mass != mass_)
    {
        mass_ = mass;
        UpdateMassAndInertia();
    }
}

void Urho3DPhysX::RigidBody::SetCenterOfMass(const Vector3 & center)
{
    if (centerOfMass_ != center)
    {
        centerOfMass_ = center;
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
        {
            body->setCMassLocalPose(ToPxTransform(center));
            UpdateMassAndInertia();
        }
    }
}

void Urho3DPhysX::RigidBody::SetLinearDamping(float dumping)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            body->setLinearDamping(Max(0.0f, dumping));
    }
}

float Urho3DPhysX::RigidBody::GetLinearDamping() const
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        return body ? body->getLinearDamping() : 0.0f;
    }
    return 0.0f;
}

void Urho3DPhysX::RigidBody::SetAngularDamping(float dumping)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            body->setAngularDamping(Max(0.0f, dumping));
    }
}

float Urho3DPhysX::RigidBody::GetAngularDamping() const
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        return body ? body->getAngularDamping() : 0.0f;
    }
    return 0.0f;
}

void Urho3DPhysX::RigidBody::SetMaxLinearVelocity(float value)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            body->setMaxLinearVelocity(value);
    }
}

float Urho3DPhysX::RigidBody::GetMaxLinearVelocity() const
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            return body->getMaxLinearVelocity();
    }
    return 0.0f;
}

void Urho3DPhysX::RigidBody::SetLinearVelocity(const Vector3 & value)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            body->setLinearVelocity(ToPxVec3(value));
    }
}

Vector3 Urho3DPhysX::RigidBody::GetLinearVelocity() const
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            return ToVector3(body->getLinearVelocity());
    }
    return Vector3();
}

void Urho3DPhysX::RigidBody::SetMaxAngularVelocity(float value)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            body->setMaxAngularVelocity(value);
    }
}

float Urho3DPhysX::RigidBody::GetMaxAngularVelocity() const
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            return body->getMaxAngularVelocity();
    }
    return 0.0f;
}

void Urho3DPhysX::RigidBody::SetAngularVelocity(const Vector3 & value)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            body->setAngularVelocity(ToPxVec3(value));
    }
}

Vector3 Urho3DPhysX::RigidBody::GetAngularVelocity() const
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
            return ToVector3(body->getAngularVelocity());
    }
    return Vector3();
}

void Urho3DPhysX::RigidBody::UpdateMassAndInertia()
{
    PxRigidBody* body = actor_->is<PxRigidBody>();
    if (body)
    {
        PxRigidBodyExt::setMassAndUpdateInertia(*body, mass_, &(ToPxVec3(centerOfMass_)));
        isDirty_ = false;
    }
}

void Urho3DPhysX::RigidBody::ApplyForce(const Vector3 & force, const Vector3 & pos, TransformSpace space)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
        {
            switch (space)
            {
            case Urho3D::TS_LOCAL:
                PxRigidBodyExt::addForceAtLocalPos(*body, ToPxVec3(force), ToPxVec3(pos));
                break;
            case Urho3D::TS_PARENT:
                PxRigidBodyExt::addForceAtPos(*body, ToPxVec3(force), ToPxVec3(node_->GetPosition() + pos));
                break;
            case Urho3D::TS_WORLD:
                PxRigidBodyExt::addForceAtPos(*body, ToPxVec3(force), ToPxVec3(pos));
                break;
            default:
                break;
            }
        }
    }
}

void Urho3DPhysX::RigidBody::ApplyImpulse(const Vector3 & force, const Vector3 & pos, TransformSpace space)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
        {
            switch (space)
            {
            case Urho3D::TS_LOCAL:
                PxRigidBodyExt::addForceAtLocalPos(*body, ToPxVec3(force), ToPxVec3(pos), PxForceMode::eIMPULSE);
                break;
            case Urho3D::TS_PARENT:
                PxRigidBodyExt::addForceAtPos(*body, ToPxVec3(force), ToPxVec3(node_->GetPosition() + pos), PxForceMode::eIMPULSE);
                break;
            case Urho3D::TS_WORLD:
                PxRigidBodyExt::addForceAtPos(*body, ToPxVec3(force), ToPxVec3(pos), PxForceMode::eIMPULSE);
                break;
            default:
                break;
            }
        }
    }
}

void Urho3DPhysX::RigidBody::ApplyVelocityChange(const Vector3 & force, const Vector3 & pos, TransformSpace space)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
        {
            switch (space)
            {
            case Urho3D::TS_LOCAL:
                PxRigidBodyExt::addForceAtLocalPos(*body, ToPxVec3(force), ToPxVec3(pos), PxForceMode::eVELOCITY_CHANGE);
                break;
            case Urho3D::TS_PARENT:
                PxRigidBodyExt::addForceAtPos(*body, ToPxVec3(force), ToPxVec3(node_->GetPosition() + pos), PxForceMode::eVELOCITY_CHANGE);
                break;
            case Urho3D::TS_WORLD:
                PxRigidBodyExt::addForceAtPos(*body, ToPxVec3(force), ToPxVec3(pos), PxForceMode::eVELOCITY_CHANGE);
                break;
            default:
                break;
            }
        }
    }
}

void Urho3DPhysX::RigidBody::ApplyAcceleration(const Vector3 & force, const Vector3 & pos, TransformSpace space)
{
    if (actor_)
    {
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
        {
            switch (space)
            {
            case Urho3D::TS_LOCAL:
                PxRigidBodyExt::addForceAtLocalPos(*body, ToPxVec3(force), ToPxVec3(pos), PxForceMode::eACCELERATION);
                break;
            case Urho3D::TS_PARENT:
                PxRigidBodyExt::addForceAtPos(*body, ToPxVec3(force), ToPxVec3(node_->GetPosition() + pos), PxForceMode::eACCELERATION);
                break;
            case Urho3D::TS_WORLD:
                PxRigidBodyExt::addForceAtPos(*body, ToPxVec3(force), ToPxVec3(pos), PxForceMode::eACCELERATION);
                break;
            default:
                break;
            }
        }
    }
}

void Urho3DPhysX::RigidBody::SetCCDEnabled(bool enable)
{
    if (ccdEnabled_ != enable)
    {
        ccdEnabled_ = enable;
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
        {
            body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, enable);
        }
    }
}

void Urho3DPhysX::RigidBody::SetKinematic(bool kinematic)
{
    if (kinematic_ != kinematic)
    {
        kinematic_ = kinematic;
        PxRigidBody* body = actor_->is<PxRigidBody>();
        if (body)
        {
            body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, kinematic);
            if (!kinematic_ && ccdEnabled_)
            {
                //reset ccd - it's not supported with kinematic bodies and flag gets cleared
                body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
            }
        }
    }
}

void Urho3DPhysX::RigidBody::OnNodeSet(Node * node)
{
    RigidActor::OnNodeSet(node);
    //update mass and inertia in case that any shape was added
    UpdateMassAndInertia();
    if (actor_ && actor_->getNbShapes())
        UpdateMassAndInertia();
}
