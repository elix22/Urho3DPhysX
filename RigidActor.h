#pragma once

#include <Urho3D/Scene/Component.h>
#include <PxRigidActor.h>

using namespace Urho3D;
using namespace physx;

namespace Urho3DPhysX
{
    class PhysXScene;
    class CollisionShape;
    class Joint;

    class __declspec(dllexport) RigidActor : public Component
    {
        URHO3D_OBJECT(RigidActor, Component);
        friend class Joint;
    public:
        RigidActor(Context* context);
        virtual ~RigidActor();

        static void RegisterObject(Context* context);
        ///
        void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;
        ///
        void OnSetEnabled() override;
        ///Attach collision shape
        virtual bool AttachShape(CollisionShape* shape, bool updateMassAndInteria = true);
        ///Apply world transform from the actor
        void ApplyWorldTransformFromActor();
        ///Apply world transform to the node
        void ApplyWorldTransform(const Vector3& worldPosition, const Quaternion& worldRotation);
        ///Apply world transform to the node
        void ApplyWorldTransform(const PxVec3& worldPosition, const PxQuat& worldRotation);
        ///update actor transform from node transform
        void UpdateTransformFromNode(bool awake = true);
        ///set actor transform
        void SetTransform(const Vector3& position, const Quaternion& rotation, bool awake = true);
        ///set actor transform
        void SetTransform(const Matrix3x4& matrix, bool awake = true);
        ///
        bool IsApplyingTransform() const { return isApplyingTransform_; }
        ///
        void RemoveFromScene();
        ///remove actor
        void ReleaseActor();
        ///
        PxRigidActor* GetActor() { return actor_; }
    protected:
        virtual void OnJointAdded(Joint* joint) {};
        virtual void OnJointRemoved(Joint* joint) {};
        void OnSceneSet(Scene* scene) override;
        void OnNodeSet(Node* node) override;
        void OnMarkedDirty(Node* node) override;
        PxRigidActor* actor_;
        WeakPtr<PhysXScene> pxScene_;
        PODVector<Joint*> joints_;
        bool isApplyingTransform_;
    private:
        ///
        void AddJoint(Joint* joint);
        ///
        void RemoveJoint(Joint* joint);
    };
}
