#pragma once

#include "PhysXUtils.h"
#include <Urho3D/Scene/Component.h>
#include <PxShape.h>

namespace Urho3D
{
    class Model;
}

using namespace Urho3D;
using namespace physx;

namespace Urho3DPhysX
{
    class RigidActor;
    class PhysXMaterial;

    enum URHOPX_API PhysXShapeType
    {
        BOX_SHAPE = 0,
        SPHERE_SHAPE,
        PLANE_SHAPE,
        CAPSULE_SHAPE,
        CONVEXMESH_SHAPE,
        TRIANGLEMESH_SHAPE
    };

    class URHOPX_API CollisionShape : public Component
    {
        URHO3D_OBJECT(CollisionShape, Component);
        friend class RigidActor;
    public:
        CollisionShape(Context* context);
        ~CollisionShape();

        void ApplyAttributes() override;

        static void RegisterObject(Context* context);
        ///
        void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;
        void OnNodeSet(Node* node) override;
        ///
        void OnSetEnabled() override;
        ///(re)create shape. Created shape will be exclusive (TODO: add support for shared shapes).
        void UpdateShape();
        void UpdateShapePose();
        void ReleaseShape();
        PhysXMaterial* GetMaterial() { return material_; }
        void SetMaterial(PhysXMaterial* material);
        void SetDefaultMaterial();
        PxShape* GetShape() { return shape_; }
        void SetShapeType(PhysXShapeType shape);
        PhysXShapeType GetShapeType() const { return shapeType_; }
        void SetPosition(const Vector3& position);
        const Vector3& GetPosition() const { return position_; }
        void SetRotation(const Quaternion& rotation);
        const Quaternion& GetRotation() const { return rotation_; }
        void SetSize(const Vector3& size);
        const Vector3& GetSize() const { return size_; }
        void SetPlaneNormal(const Vector3& normal);
        const Vector3& GetPlaneNormal() const { return planeNormal_; }
        ///Set as box
        void SetBox(const Vector3& size = Vector3::ONE, const Vector3& position = Vector3::ZERO, const Quaternion& rotation = Quaternion::IDENTITY);
        ///Set as sphere
        void SetSphere(float diamater = 1.0f, const Vector3& position = Vector3::ZERO, const Quaternion& rotation = Quaternion::IDENTITY);
        /// Set as capsule. By default rotated to vertical orientation (for characters etc.)
        void SetCapsule(float diamater = 1.0f, float height = 1.0f, const Vector3& position = Vector3::ZERO, const Quaternion& rotation = Quaternion(90.0f, Vector3::FORWARD));
        ///
        bool CreateTriangleMesh();
        ///
        bool CreateConvexMesh();
        ///
        void SetTrigger(bool trigger);
        ///
        bool IsTrigger() const { return trigger_; }
        ///
        void SetCollisionLayer(unsigned layer);
        ///
        unsigned GetCollisionLayer() const { return collisionLayer_; }
        ///
        void SetCollisionMask(unsigned mask);
        ///
        unsigned GetCollisionMask() const { return collisionMask_; }
        ///
        void SetCustomModel(Model* model);
        ///
        Model* GetCustomModel() { return customModel_; }
        ///
        void SetCustomModelAttr(const ResourceRef& value);
        ///
        unsigned GetModelLODLevel() const { return modelLodLevel_; }
        ///
        void SetModelLODLevel(unsigned value);
        ///
        ResourceRef GetCustomModelAttr() const;
        ///
        void SetMaterialAttr(const ResourceRef& material);
        ///
        ResourceRef GetMaterialAttr() const;
    private:
        void OnMarkedDirty(Node* node) override;
        void SetActor(RigidActor* actor);
        void UpdateSize();
        void UpdateBoxSize();
        void UpdateSphereSize();
        void UpdateCapsuleSize();
        void UpdateTriangleMeshScale();
        void UpdateConvexMeshScale();
        Model* FindSourceModel();
        PxShape* shape_;
        WeakPtr<RigidActor> rigidActor_;
        Vector3 position_;
        Quaternion rotation_;
        Vector3 planeNormal_;
        Vector3 size_;
        Vector3 cachedWorldScale_;
        PhysXShapeType shapeType_;
        SharedPtr<PhysXMaterial> material_;
        bool trigger_;
        unsigned collisionLayer_;
        unsigned collisionMask_;
        //model
        SharedPtr<Model> customModel_;
        //model lod level
        unsigned modelLodLevel_;
    };
}
