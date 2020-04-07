#pragma once
#include "PhysXUtils.h"
#include "PhysXEvents.h"
#include <Urho3D/Scene/Component.h>
#include <characterkinematic/PxController.h>
#include <characterkinematic/PxControllerBehavior.h>

using namespace Urho3D;
using namespace physx;

namespace Urho3DPhysX
{
    class PhysXScene;
    class PhysXMaterial;

    enum URHOPX_API ControllerType
    {
        CAPSULE_CONTROLLER,
        BOX_CONTROLLER
    };

    enum URHOPX_API ControllerNonWalkableMode
    {
        PREV_CLIMBING,
        PREV_CLIMBING_FORCE_SLIDING
    };

    class URHOPX_API KinematicController : public Component
    {
        URHO3D_OBJECT(KinematicController, Component);
        friend class ControllerHitCallback;
    public:
        KinematicController(Context* context);
        ~KinematicController();

        static void RegisterObject(Context* context);

        void ApplyAttributes() override;

        void DrawDebugGeometry(DebugRenderer* debug, bool depthTest = true) override;

        PxController* GetController() { return controller_; }
        ///
        bool CreateController();
        ///
        unsigned char Move(const Vector3& displ, float minDist, float timeStep);
        ///
        unsigned char MoveWithGravity(const Vector3& displ, float minDist, float timeStep);
        ///
        Vector3 GetLinearVelocity() const;
        ///
        Vector3 GetAngularVelocity() const;
        ///
        Vector3 GetFootPosition() const;
        ///
        const Vector3& GetPosition() const { return position_; }
        ///
        void SetPosition(const Vector3& position);
        ///
        float GetStepOffset() const { return stepOffset_; }
        ///
        void SetStepOffset(float offset);
        ///
        float GetContactOffset() const { return contactOffset_; }
        ///
        void SetContactOffset(float offset);
        ///
        const Vector3& GetUpDirection() const { return upDirection_; }
        ///
        void SetUpDirection(const Vector3& direction);
        ///
        float GetSlopeLimit() const { return slopeLimit_; }
        ///
        void SetSlopeLimit(float limit);
        ///
        float GetDensity() const { return density_; }
        ///
        void SetDensity(float value);
        ///
        float GetCapsuleHeight() const { return capsuleHeight_; }
        ///
        void SetCapsuleHeight(float height);
        ///
        float GetCapsuleRadius() const { return capsuleRadius_; }
        ///
        void SetCapsuleRadius(float radius);
        ///Set capsule size from Vector2 (x = radius, y = heigth)
        void SetCapsuleSize(const Vector2& size);
        ///
        float GetBoxHalfHeight() const { return boxHalfHeight_; }
        ///
        void SetBoxHalfHeight(float halfHeight);
        ///
        float GetBoxHalfSideExtend() const { return boxHalfSideExtend_; }
        ///
        void SetBoxHalfSideExtend(float halfExtend);
        ///
        float GetBoxHalfForwardExtend() const { return boxHalfForwardExtend_; }
        ///
        void SetBoxHalfForwadExtend(float halfExtend);
        ///Set box half size from Vector3 (x = side extend, y = height, z - forward extend)
        void SetBoxHalfSize(const Vector3& halfSize);
        ///Set box size from Vector3 (x = side extend, y = height, z - forward extend)
        void SetBoxSize(const Vector3& size);
        ///
        ControllerNonWalkableMode GetNonWalkableMode() const { return nonWalkableMode_; }
        ///
        void SetNonWalkableMode(ControllerNonWalkableMode mode);
        ///
        PhysXMaterial* GetMaterial() { return material_; }
        ///
        void SetMaterial(PhysXMaterial* material);
        ///
        ControllerType GetControllerType() const { return controllerType_; }
        ///
        void SetControllerType(ControllerType type);
        ///
        unsigned GetCollisionLayer() const { return collisionLayer_; }
        ///
        void SetCollisionLayer(unsigned layer);
        ///
        unsigned GetCollisionMask() const { return collisionMask_; }
        ///
        void SetCollisionMask(unsigned mask);
        ///
        void UpdateCollisionLayerAndMask();
        ///
        void InvalidateCache();
        ///
        void Resize(float height);
        ///
        void UpdatePositionFromNode();
        ///
        bool NodeFromFoot() const { return nodeFromFoot_; }
        ///
        void UseNodeFromFoot(bool value) { nodeFromFoot_ = value; }
        ///
        void SetCCTFilterCallback(PxControllerFilterCallback* callback);
        ///
        void SetControllerHitCallback(PxUserControllerHitReport* callback);
        ///
        void SetBehaviorControllerCallback(PxControllerBehaviorCallback* callback);

    private:
        void OnSceneSet(Scene* scene) override;
        void OnNodeSet(Node* node) override;
        void OnMarkedDirty(Node* node) override;
        void OnShapeHit(const PxControllerShapeHit& hit);
        void OnControllerHit(const PxControllersHit& hit);
        void UpdateNodePosition();
        void ReleaseController();
        PxController* controller_;
        PhysXScene* pxScene_;

        ControllerType controllerType_;

        Vector3 position_;
        float stepOffset_;
        float contactOffset_;
        Vector3 upDirection_;
        float slopeLimit_;
        float density_;
        //capsule controller
        float capsuleHeight_;
        float capsuleRadius_;
        //box controller
        float boxHalfHeight_;
        float boxHalfSideExtend_;
        float boxHalfForwardExtend_;

        unsigned collisionLayer_;
        unsigned collisionMask_;

        bool nodeFromFoot_;

        bool recreatingNeeded_;

        ControllerNonWalkableMode nonWalkableMode_;
        SharedPtr<PhysXMaterial> material_;

        PxControllerFilters filters_;
        PxFilterData filterData_;
        PxUserControllerHitReport* hitCallback_;
        PxControllerBehaviorCallback* behaviorCallback_;

        Vector3 lastPosition_;
    };
}
