#include "CollisionShape.h"
#include "Physics.h"
#include "PhysXUtils.h"
#include "PhysXMaterial.h"
#include "StaticBody.h"
#include "DynamicBody.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/DebugRenderer.h>

namespace Urho3DPhysX
{
    static const unsigned DEF_COLLISION_LAYER = 0x1;
    static const unsigned DEF_COLLISION_MASK = M_MAX_UNSIGNED;

    static const char* collisionShapesNames[] =
    {
        "Box",
        "Sphere",
        "Plane",
        "Capsule",
        "ConvexMesh",
        "TriangleMesh",
        //heightfield,
        nullptr
    };
}

Urho3DPhysX::CollisionShape::CollisionShape(Context * context) : Component(context),
rigidActor_(nullptr),
position_(Vector3::ZERO),
rotation_(Quaternion::IDENTITY),
planeNormal_(Vector3::UP),
size_(Vector3::ONE),
cachedWorldScale_(Vector3::ONE),
shape_(nullptr),
shapeType_(BOX_SHAPE),
trigger_(false),
collisionLayer_(DEF_COLLISION_LAYER),
collisionMask_(DEF_COLLISION_MASK),
customModel_(nullptr),
modelLodLevel_(0),
material_(nullptr)
{    
}

Urho3DPhysX::CollisionShape::~CollisionShape()
{
    ReleaseShape();
}

void Urho3DPhysX::CollisionShape::ApplyAttributes()
{
}

void Urho3DPhysX::CollisionShape::RegisterObject(Context * context)
{
    context->RegisterFactory<CollisionShape>("PhysX");
    URHO3D_ACCESSOR_ATTRIBUTE("Position", GetPosition, SetPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Rotation", GetRotation, SetRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Size", GetSize, SetSize, Vector3, Vector3::ONE, AM_DEFAULT);
    URHO3D_ENUM_ACCESSOR_ATTRIBUTE("ShapeType", GetShapeType, SetShapeType, PhysXShapeType, collisionShapesNames, BOX_SHAPE, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Plane normal", GetPlaneNormal, SetPlaneNormal, Vector3, Vector3::UP, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Trigger", IsTrigger, SetTrigger, bool, false, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Collision layer", GetCollisionLayer, SetCollisionLayer, unsigned, DEF_COLLISION_LAYER, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Collision mask", GetCollisionMask, SetCollisionMask, unsigned, DEF_COLLISION_MASK, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Material", GetMaterialAttr, SetMaterialAttr, ResourceRef, ResourceRef(PhysXMaterial::GetTypeStatic()), AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Custom model", GetCustomModelAttr, SetCustomModelAttr, ResourceRef, ResourceRef(Model::GetTypeStatic()), AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Model LOD level", GetModelLODLevel, SetModelLODLevel, unsigned, 0, AM_DEFAULT);
}

void Urho3DPhysX::CollisionShape::DrawDebugGeometry(DebugRenderer * debug, bool depthTest)
{
    if (debug && shape_)
    {
        Vector3 worldPosition = node_->GetWorldPosition() + position_;
        Quaternion worldRotation = node_->GetWorldRotation() * rotation_;
        Color color = Color::GRAY;
        if (rigidActor_)
        {
            StaticBody* staticBody = rigidActor_->Cast<StaticBody>();
            if (staticBody)
                color = Color::GREEN;
            else
            {
                DynamicBody* dynamicBody = rigidActor_->Cast<DynamicBody>();
                if (dynamicBody)
                {
                    color = dynamicBody->IsSleeping() ? Color::WHITE : Color::MAGENTA;
                }
            }
        }
        switch (shapeType_)
        {
        case Urho3DPhysX::BOX_SHAPE:
            {
                Vector3 halfSize = size_ * cachedWorldScale_ * 0.5f;
                debug->AddBoundingBox(BoundingBox(-halfSize, halfSize), Matrix3x4(worldPosition, worldRotation.Normalized(), 1.0f), color, depthTest);
            }
            break;
        case Urho3DPhysX::SPHERE_SHAPE:
            {
                float radius = size_.x_ * cachedWorldScale_.x_ * 0.5f;
                debug->AddCircle(worldPosition, node_->GetWorldDirection(), radius, color, 32, depthTest);
                debug->AddCircle(worldPosition, (node_->GetWorldRotation() * Vector3::LEFT).Normalized(), radius, color, 32, depthTest);
                debug->AddCircle(worldPosition, (node_->GetWorldRotation() * Vector3::UP).Normalized(), radius, color, 32, depthTest); 
            }
            break;
        case Urho3DPhysX::PLANE_SHAPE:
            break;
        case Urho3DPhysX::CAPSULE_SHAPE:
        {
            //https://discourse.urho3d.io/t/is-there-a-horizontal-tube-in-debug-renderer/2253/16
            float radius = size_.x_ * cachedWorldScale_.x_ * 0.5f;
            float halfHeight = size_.y_ * cachedWorldScale_.y_ * 0.5f;
            Sphere sphere(Vector3::ZERO, radius);
            Vector3 halfLengthVec = worldRotation * Vector3(halfHeight, 0, 0);
            for (unsigned j = 0; j < 180; j += 45)
            {
                for (unsigned i = 0; i < 180; i += 45)
                {
                    Vector3 p1 = worldRotation * sphere.GetPoint(i, j) + halfLengthVec + worldPosition;
                    Vector3 p2 = worldRotation * sphere.GetPoint(i + 45, j) + halfLengthVec + worldPosition;
                    Vector3 p3 = worldRotation * sphere.GetPoint(i, j + 45) + halfLengthVec + worldPosition;
                    Vector3 p4 = worldRotation * sphere.GetPoint(i + 45, j + 45) + halfLengthVec + worldPosition;

                    debug->AddLine(p1, p2, color, depthTest);
                    debug->AddLine(p1, p3, color, depthTest);
                    debug->AddLine(p2, p4, color, depthTest);
                }

                for (unsigned i = 180; i < 360; i += 45)
                {
                    Vector3 p1 = worldRotation * sphere.GetPoint(i, j) - halfLengthVec + worldPosition;
                    Vector3 p2 = worldRotation * sphere.GetPoint(i + 45, j) - halfLengthVec + worldPosition;
                    Vector3 p3 = worldRotation * sphere.GetPoint(i, j + 45) - halfLengthVec + worldPosition;
                    Vector3 p4 = worldRotation * sphere.GetPoint(i + 45, j + 45) - halfLengthVec + worldPosition;

                    debug->AddLine(p1, p2, color, depthTest);
                    debug->AddLine(p1, p3, color, depthTest);
                    debug->AddLine(p2, p4, color, depthTest);
                }

                Vector3 p1 = worldRotation * sphere.GetPoint(0, j) + halfLengthVec + worldPosition;
                Vector3 p2 = worldRotation * sphere.GetPoint(0, j) - halfLengthVec + worldPosition;
                debug->AddLine(p1, p2, depthTest);
                Vector3 p3 = worldRotation * sphere.GetPoint(0, j + 180) + halfLengthVec + worldPosition;
                Vector3 p4 = worldRotation * sphere.GetPoint(0, j + 180) - halfLengthVec + worldPosition;
                debug->AddLine(p3, p4, depthTest);
            }
        }
            break;
        case Urho3DPhysX::CONVEXMESH_SHAPE:
            break;
        case Urho3DPhysX::TRIANGLEMESH_SHAPE:
            break;
        default:
            break;
        }        
    }
}

void Urho3DPhysX::CollisionShape::OnNodeSet(Node * node)
{
    if (node)
    {
        node->AddListener(this);
        UpdateShape();
    }
}

void Urho3DPhysX::CollisionShape::OnSetEnabled()
{
    if (shape_)
    {
        if (trigger_)
        {
            shape_->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
            shape_->setFlag(PxShapeFlag::eTRIGGER_SHAPE, enabled_);
        }
        else
        {
            shape_->setFlag(PxShapeFlag::eSIMULATION_SHAPE, enabled_);
            if (rigidActor_)
            {
                RigidBody* rigidBody = rigidActor_->Cast<RigidBody>();
                if (rigidBody)
                    rigidBody->UpdateMassAndInertia();
            }
        }
    }
}

void Urho3DPhysX::CollisionShape::UpdateShape()
{
    ReleaseShape();
    if (node_)
    {
        auto* physics = GetSubsystem<Physics>();
        if (!physics)
        {
            return;
        }
        auto* px = physics->GetPhysics();
        if (!px)
        {
            return;
        }
        if (!material_)
            SetDefaultMaterial();
        PxMaterial* mat = material_->GetMaterial();
        if (node_)
        {
            cachedWorldScale_ = node_->GetWorldScale();
            switch (shapeType_)
            {
            case Urho3DPhysX::BOX_SHAPE:
                shape_ = px->createShape(PxBoxGeometry(ToPxVec3(size_ * cachedWorldScale_ * 0.5f)), *mat, true);
                break;
            case Urho3DPhysX::SPHERE_SHAPE:
                shape_ = px->createShape(PxSphereGeometry(size_.x_ * cachedWorldScale_.x_ * 0.5f), *mat, true);
                break;
            case Urho3DPhysX::PLANE_SHAPE:
                shape_ = px->createShape(PxPlaneGeometry(), *mat, true);
                break;
            case Urho3DPhysX::CAPSULE_SHAPE:
                shape_ = px->createShape(PxCapsuleGeometry(size_.x_ * cachedWorldScale_.x_ * 0.5f, size_.y_ * cachedWorldScale_.y_ * 0.5f), *mat, true);
                break;
            case Urho3DPhysX::CONVEXMESH_SHAPE:
                if (!CreateConvexMesh())
                {
                    //TODO: handle this
                }
                break;
            case Urho3DPhysX::TRIANGLEMESH_SHAPE:
                if (!CreateTriangleMesh())
                {
                    //TODO: handle this
                }
                break;
            default:
                break;
            }
            //check if shape creation failed
            if (!shape_)
            {
                //TODO: react to this
            }
            else
            {
                PxFilterData filter;
                filter.word0 = collisionLayer_;
                filter.word1 = collisionMask_;
                shape_->setSimulationFilterData(filter);
                shape_->setQueryFilterData(filter);
                SetMaterial(material_);
                UpdateShapePose();
                shape_->userData = this;
                RigidActor* actor = node_->GetDerivedComponent<RigidActor>();
                if (actor)
                    actor->AttachShape(this);
            }
        }
    }
}

void Urho3DPhysX::CollisionShape::UpdateShapePose()
{
    if (shape_)
    {
        if (shapeType_ == PLANE_SHAPE)
        {
            shape_->setLocalPose(PxTransformFromPlaneEquation(PxPlane(ToPxVec3(position_), ToPxVec3(planeNormal_))));
        }
        else
        {
            shape_->setLocalPose(PxTransform(ToPxVec3(position_), ToPxQuat(rotation_)));
        }
        if (rigidActor_)
        {
            RigidBody* rigidBody = rigidActor_->Cast<RigidBody>();
            if (rigidBody)
                rigidBody->UpdateMassAndInertia();
        }
    }
}

void Urho3DPhysX::CollisionShape::ReleaseShape()
{
    if (shape_)
    {
        if (rigidActor_)
        {
            rigidActor_->GetActor()->detachShape(*shape_);
        }
        shape_->userData = nullptr;
        shape_->release();
        shape_ = nullptr;
    }
}

void Urho3DPhysX::CollisionShape::SetMaterial(PhysXMaterial * material)
{
    if (!material)
    {
        SetDefaultMaterial();
    }
    else
    {
        material_ = SharedPtr<PhysXMaterial>(material);
        PxMaterial* mat[1];
        mat[0] = material_->GetMaterial();
        if(shape_)
            shape_->setMaterials(mat, 1);        
    }
}

void Urho3DPhysX::CollisionShape::SetDefaultMaterial()
{
    auto* physics = GetSubsystem<Physics>();
    SetMaterial(physics->GetDefaultMaterial());
}

void Urho3DPhysX::CollisionShape::SetShapeType(PhysXShapeType shape)
{
    if (shapeType_ != shape)
    {
        shapeType_ = shape;
        UpdateShape();
    }
}

void Urho3DPhysX::CollisionShape::SetPosition(const Vector3 & position)
{
    position_ = position;
    UpdateShapePose();
}

void Urho3DPhysX::CollisionShape::SetRotation(const Quaternion & rotation)
{
    rotation_ = rotation;
    UpdateShapePose();
}

void Urho3DPhysX::CollisionShape::SetSize(const Vector3 & size)
{
    if (size != size_)
    {
        size_ = size;
        UpdateSize();
    }
}

void Urho3DPhysX::CollisionShape::SetPlaneNormal(const Vector3 & normal)
{
    planeNormal_ = normal;
    if (shapeType_ = PLANE_SHAPE)
        UpdateShapePose();
}

void Urho3DPhysX::CollisionShape::SetBox(const Vector3 & size, const Vector3 & position, const Quaternion & rotation)
{
    shapeType_ = BOX_SHAPE;
    size_ = size;
    position_ = position;
    rotation_ = rotation;
    UpdateShape();
}

void Urho3DPhysX::CollisionShape::SetSphere(float radius, const Vector3 & position, const Quaternion & rotation)
{
    shapeType_ = SPHERE_SHAPE;
    size_ = Vector3(radius, radius, radius);
    position_ = position;
    rotation_ = rotation;
    UpdateShape();
}

void Urho3DPhysX::CollisionShape::SetCapsule(float diamater, float height, const Vector3 & position, const Quaternion & rotation)
{
    shapeType_ = CAPSULE_SHAPE;
    size_ = Vector3(diamater, height, diamater);
    position_ = position;
    rotation_ = rotation;
    UpdateShape();
}

bool Urho3DPhysX::CollisionShape::CreateTriangleMesh()
{
    if (shape_)
        ReleaseShape();
    Model* sourceModel = FindSourceModel();
    if (sourceModel)
    {
        auto* physics = GetSubsystem<Physics>();
        PxTriangleMesh* triMesh = physics->GetOrCreateTriangleMesh(sourceModel, modelLodLevel_);
        if (triMesh)
        {
            cachedWorldScale_ = node_->GetWorldScale();
            PxMeshScale scale = ToPxVec3(cachedWorldScale_);
            PxTriangleMeshGeometry geom(triMesh, scale);
            shape_ = physics->GetPhysics()->createShape(geom, *material_->GetMaterial(), true);
            if (shape_)
                return true;
        }
    }
    return false;
}

bool Urho3DPhysX::CollisionShape::CreateConvexMesh()
{
    if (shape_)
        ReleaseShape();
    Model* sourceModel = FindSourceModel();
    if (sourceModel)
    {
        auto* physics = GetSubsystem<Physics>();
        PxConvexMesh* convexMesh = physics->GetOrCreateConvexMesh(sourceModel, modelLodLevel_);
        if (convexMesh)
        {
            cachedWorldScale_ = node_->GetWorldScale();
            PxMeshScale scale = ToPxVec3(cachedWorldScale_);
            PxConvexMeshGeometry geom(convexMesh, scale);
            shape_ = physics->GetPhysics()->createShape(geom, *material_->GetMaterial(), true);
            if (shape_)
                return true;
        }
    }
    return false;
}

void Urho3DPhysX::CollisionShape::SetTrigger(bool trigger)
{
    if (trigger_ != trigger)
    {
        trigger_ = trigger;
        if (shape_)
        {
            shape_->setFlag(PxShapeFlag::eSIMULATION_SHAPE, !trigger_);
            shape_->setFlag(PxShapeFlag::eTRIGGER_SHAPE, trigger_);
            if (rigidActor_)
            {
                RigidBody* rigidBody = rigidActor_->Cast<RigidBody>();
                if (rigidBody)
                    rigidBody->UpdateMassAndInertia();
            }
        }
    }
}

void Urho3DPhysX::CollisionShape::SetCollisionLayer(unsigned layer)
{
    if (collisionLayer_ != layer)
    {
        collisionLayer_ = layer;
        if (shape_)
        {
            PxFilterData filter;
            filter.word0 = collisionLayer_;
            filter.word1 = collisionMask_;
            shape_->setSimulationFilterData(filter);
            shape_->setQueryFilterData(filter);
        }
    }
}

void Urho3DPhysX::CollisionShape::SetCollisionMask(unsigned mask)
{
    if (collisionMask_ != mask)
    {
        collisionMask_ = mask;
        if (shape_)
        {
            PxFilterData filter;
            filter.word0 = collisionLayer_;
            filter.word1 = collisionMask_;
            shape_->setSimulationFilterData(filter);
            shape_->setQueryFilterData(filter);
        }
    }
}

void Urho3DPhysX::CollisionShape::SetCustomModel(Model* model)
{
    if (model != customModel_)
    {
        customModel_ = SharedPtr<Model>(model);
        if (customModel_ && (shapeType_ == TRIANGLEMESH_SHAPE || shapeType_ == CONVEXMESH_SHAPE))
        {
            UpdateShape();
        }
    }
}

void Urho3DPhysX::CollisionShape::SetCustomModelAttr(const ResourceRef & value)
{
    SetCustomModel(GetSubsystem<ResourceCache>()->GetResource<Model>(value.name_));
}

void Urho3DPhysX::CollisionShape::SetModelLODLevel(unsigned value)
{
    if (value != modelLodLevel_)
    {
        modelLodLevel_ = value;
        if (shapeType_ == TRIANGLEMESH_SHAPE || shapeType_ == CONVEXMESH_SHAPE)
            UpdateShape();
    }
}

ResourceRef Urho3DPhysX::CollisionShape::GetCustomModelAttr() const
{
    return GetResourceRef(customModel_, Model::GetTypeStatic());
}

void Urho3DPhysX::CollisionShape::SetMaterialAttr(const ResourceRef & material)
{
    SetMaterial(GetSubsystem<ResourceCache>()->GetResource<PhysXMaterial>(material.name_));
}

ResourceRef Urho3DPhysX::CollisionShape::GetMaterialAttr() const
{
    return GetResourceRef(material_, PhysXMaterial::GetTypeStatic());
}

void Urho3DPhysX::CollisionShape::OnMarkedDirty(Node * node)
{
    if (shape_)
    {
        if (rigidActor_ && rigidActor_->IsApplyingTransform())
            return;
        if (cachedWorldScale_ != node->GetWorldScale())
        {
            UpdateSize();
        }
    }
}

void Urho3DPhysX::CollisionShape::SetActor(RigidActor * actor)
{
    if (actor != rigidActor_)
    {
        if (rigidActor_)
        {
            /*TODO: this situation is not desired, but if happend, require some clean up*/
        }
        rigidActor_ = WeakPtr<RigidActor>(actor);
    }
}

void Urho3DPhysX::CollisionShape::UpdateSize()
{
    cachedWorldScale_ = node_->GetWorldScale();
    switch (shapeType_)
    {
    case Urho3DPhysX::BOX_SHAPE:
        UpdateBoxSize();
        break;
    case Urho3DPhysX::SPHERE_SHAPE:
        UpdateSphereSize();
        break;
    case Urho3DPhysX::PLANE_SHAPE:
        //plane is not affected by scale
        break;
    case Urho3DPhysX::CAPSULE_SHAPE:
        UpdateCapsuleSize();
        break;
    case Urho3DPhysX::CONVEXMESH_SHAPE:
        UpdateConvexMeshScale();
        break;
    case Urho3DPhysX::TRIANGLEMESH_SHAPE:
        UpdateTriangleMeshScale();
        break;
    default:
        break;
    }
    if(rigidActor_)
    {
        RigidBody* rigidBody = rigidActor_->Cast<RigidBody>();
        if (rigidBody)
            rigidBody->UpdateMassAndInertia();
    }        
}

void Urho3DPhysX::CollisionShape::UpdateBoxSize()
{
    if (shape_)
    {
        PxBoxGeometry box;
        if (shape_->getBoxGeometry(box))
        {
            box.halfExtents = ToPxVec3(size_ * cachedWorldScale_ * 0.5f);
            shape_->setGeometry(box);
        }
    }
}

void Urho3DPhysX::CollisionShape::UpdateSphereSize()
{
    if (shape_)
    {
        PxSphereGeometry sphere;
        if (shape_->getSphereGeometry(sphere))
        {
            sphere.radius = size_.x_ * cachedWorldScale_.x_ * 0.5f;
            shape_->setGeometry(sphere);
        }
    }
}

void Urho3DPhysX::CollisionShape::UpdateCapsuleSize()
{
    if (shape_)
    {
        PxCapsuleGeometry capsule;
        if (shape_->getCapsuleGeometry(capsule))
        {
            capsule.radius = size_.x_ * cachedWorldScale_.x_ * 0.5f;
            capsule.halfHeight = size_.y_ * cachedWorldScale_.y_ * 0.5f;
            shape_->setGeometry(capsule);
        }
    }
}

void Urho3DPhysX::CollisionShape::UpdateTriangleMeshScale()
{
    if (shape_)
    {
        PxTriangleMeshGeometry triangleMesh;
        if (shape_->getTriangleMeshGeometry(triangleMesh))
        {
            triangleMesh.scale = PxMeshScale(ToPxVec3(cachedWorldScale_));
            shape_->setGeometry(triangleMesh);
        }
    }
}

void Urho3DPhysX::CollisionShape::UpdateConvexMeshScale()
{
    if (shape_)
    {
        PxConvexMeshGeometry convexMesh;
        if (shape_->getConvexMeshGeometry(convexMesh))
        {
            convexMesh.scale = PxMeshScale(ToPxVec3(cachedWorldScale_));
            shape_->setGeometry(convexMesh);
        }
    }
}

Model * Urho3DPhysX::CollisionShape::FindSourceModel()
{
    if (customModel_)
        return customModel_;
    StaticModel* m = node_->GetDerivedComponent<StaticModel>();
    if (m)
    {
        return m->GetModel();
    }
    return nullptr;
}
