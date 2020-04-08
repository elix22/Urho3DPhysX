#include "KinematicController.h"
#include "PhysXScene.h"
#include "Physics.h"
#include "PhysXMaterial.h"
#include "RigidActor.h"
#include "CollisionShape.h"
#include "PhysXMaterial.h"
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <characterkinematic/PxCapsuleController.h>
#include <characterkinematic/PxBoxController.h>
#include <characterkinematic/PxControllerManager.h>
#include <foundation/PxMath.h>

static const float DEF_CTRL_SLOPE_LIMIT = 45.0f;
static const float DEF_CTRL_CONTACT_OFFSET = 0.1f;
static const float DEF_CTRL_STEP_OFFSET = 0.5f;
static const float DEF_CTRL_CAPSULE_H = 1.1f;
static const float DEF_CTRL_CAPSULE_R = 0.4f;
static const float DEF_CTRL_BOX_H = 1.0f;
static const float DEF_CTRL_BOX_SE = 0.5f;
static const float DEF_CTRL_BOX_FE = 0.5f;


Urho3DPhysX::KinematicController::KinematicController(Context* context) : Component(context),
controller_(nullptr),
controllerType_(CAPSULE_CONTROLLER),
pxScene_(nullptr),
hitCallback_(nullptr),
behaviorCallback_(nullptr),
position_(Vector3::ZERO),
upDirection_(Vector3::UP),
slopeLimit_(DEF_CTRL_SLOPE_LIMIT),
contactOffset_(DEF_CTRL_CONTACT_OFFSET),
stepOffset_(DEF_CTRL_STEP_OFFSET),
capsuleHeight_(DEF_CTRL_CAPSULE_H),
capsuleRadius_(DEF_CTRL_CAPSULE_R),
nonWalkableMode_(PREV_CLIMBING),
boxHalfHeight_(DEF_CTRL_BOX_H),
boxHalfSideExtend_(DEF_CTRL_BOX_SE),
boxHalfForwardExtend_(DEF_CTRL_BOX_FE),
recreatingNeeded_(true),
nodeFromFoot_(false),
collisionLayer_(0x1),
collisionMask_(M_MAX_UNSIGNED),
lastPosition_(Vector3::ZERO),
isMoving_(false)
{
    filters_.mFilterData = &filterData_;
    material_ = SharedPtr<PhysXMaterial>(GetSubsystem<Physics>()->GetDefaultMaterial());
}

Urho3DPhysX::KinematicController::~KinematicController()
{
    ReleaseController();
}

void Urho3DPhysX::KinematicController::RegisterObject(Context* context)
{
    context->RegisterFactory<KinematicController>("PhysX");
}

void Urho3DPhysX::KinematicController::ApplyAttributes()
{
    if (!controller_ || recreatingNeeded_)
        CreateController();
}

void Urho3DPhysX::KinematicController::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
{
    if (debug)
    {
        const Vector3& footPos = GetFootPosition();
        debug->AddSphere(Sphere(footPos, 0.1f), Color::GREEN, depthTest);
        debug->AddNode(node_, 0.5f, depthTest);
        const Vector3& vel = GetLinearVelocity();
        if (vel != Vector3::ZERO)
            debug->AddLine(node_->GetWorldPosition(), node_->GetWorldPosition() + vel.Normalized(), Color::BLUE, depthTest);
    }
}

bool Urho3DPhysX::KinematicController::CreateController()
{
    if (!GetScene())
    {
        URHO3D_LOGERROR("Node must be in the scene to create physx controller.");
        return false;
    }
    if (!pxScene_)
        pxScene_ = GetScene()->GetOrCreateComponent<PhysXScene>();
    if (controller_)
    {
        ReleaseController();
    }
    PxControllerManager* manager = pxScene_->GetControllerManager();
    switch (controllerType_)
    {
    case Urho3DPhysX::CAPSULE_CONTROLLER:
    {
        PxCapsuleControllerDesc descr;
        //descr.position = ToPxExtendedVec3(pos);
        descr.upDirection = ToPxVec3(upDirection_);
        descr.slopeLimit = Cos<float>(slopeLimit_);
        //descr.invisibleWallHeight //todo
        //descr.maxJumpHeight //todo
        descr.contactOffset = contactOffset_;
        descr.stepOffset = stepOffset_;
        //descr.density //todo
        //descr.scaleCoeff //todo
        //descr.volumeGrowth //todo
        descr.reportCallback = hitCallback_;
        descr.behaviorCallback = behaviorCallback_;
        descr.material = material_->GetMaterial();
        descr.nonWalkableMode = nonWalkableMode_ == PREV_CLIMBING ? PxControllerNonWalkableMode::ePREVENT_CLIMBING : PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
        descr.height = capsuleHeight_;
        descr.radius = capsuleRadius_;
        descr.registerDeletionListener = true;
        descr.userData = this;
        controller_ = manager->createController(descr);
        break;
    }
    case Urho3DPhysX::BOX_CONTROLLER:
    {
        PxBoxControllerDesc descr;
        //descr.position = ToPxExtendedVec3(pos);
        descr.upDirection = ToPxVec3(upDirection_);
        descr.slopeLimit = Cos<float>(slopeLimit_);
        //descr.invisibleWallHeight //todo
        //descr.maxJumpHeight //todo
        descr.contactOffset = contactOffset_;
        descr.stepOffset = stepOffset_;
        //descr.density //todo
        //descr.scaleCoeff //todo
        //descr.volumeGrowth //todo
        descr.reportCallback = hitCallback_;
        descr.behaviorCallback = behaviorCallback_;
        descr.material = material_->GetMaterial();
        descr.nonWalkableMode = nonWalkableMode_ == PREV_CLIMBING ? PxControllerNonWalkableMode::ePREVENT_CLIMBING : PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
        descr.halfHeight = boxHalfHeight_;
        descr.halfSideExtent = boxHalfSideExtend_;
        descr.halfForwardExtent = boxHalfForwardExtend_;

        descr.registerDeletionListener = true;
        descr.userData = this;
        controller_ = manager->createController(descr);
        break;
    }
    default:
        break;
    }
    if (controller_)
    {
        //leave null user data, controller pointer will be placed in shape's user data
        //this allows to determine if simulation collision is dynamic - controller
        /*PxRigidActor* actor = controller_->getActor();
        actor->userData = this;*/
        UpdatePositionFromNode();
        UpdateCollisionLayerAndMask();
        if(!filters_.mCCTFilterCallback)
            filters_.mCCTFilterCallback = pxScene_->GetControllerFilterCallback();
        recreatingNeeded_ = false;
        return true;
    }
    return false;
}

unsigned char Urho3DPhysX::KinematicController::Move(const Vector3& displ, float minDist, float timeStep)
{
    if (controller_)
    {
        isMoving_ = true;
        unsigned flags = controller_->move(ToPxVec3(displ), minDist, timeStep, filters_);
        UpdateNodePosition();
        isMoving_ = false;
        return flags;
    }
    return 0;
}

unsigned char Urho3DPhysX::KinematicController::MoveWithGravity(const Vector3& displ, float minDist, float timeStep)
{
    if (pxScene_)
        return Move(displ + (pxScene_->GetGravity()) * timeStep, minDist, timeStep);
}

Vector3 Urho3DPhysX::KinematicController::GetLinearVelocity() const
{
    if (controller_)
    {
        PxRigidDynamic* actor = controller_->getActor();
        return ToVector3(actor->getLinearVelocity());
    }
    return Vector3();
}

Vector3 Urho3DPhysX::KinematicController::GetAngularVelocity() const
{
    if (controller_)
    {
        PxRigidDynamic* actor = controller_->getActor();
        return ToVector3(actor->getAngularVelocity());
    }
    return Vector3();
}

Vector3 Urho3DPhysX::KinematicController::GetFootPosition() const
{
    if (controller_)
        return ToVector3(controller_->getFootPosition());
    return Vector3();
}

void Urho3DPhysX::KinematicController::SetPosition(const Vector3& position)
{
    position_ = position;
    if (controller_)
    {
        Vector3 worldPosition = node_ ? node_->GetWorldPosition() + position_ : position_;
        controller_->setPosition(ToPxExtendedVec3(worldPosition));
    }
}

void Urho3DPhysX::KinematicController::SetStepOffset(float offset)
{
    stepOffset_ = offset;
    if (controller_)
        controller_->setStepOffset(stepOffset_);
}

void Urho3DPhysX::KinematicController::SetContactOffset(float offset)
{
    contactOffset_ = offset;
    if (controller_)
        controller_->setContactOffset(contactOffset_);
}

void Urho3DPhysX::KinematicController::SetUpDirection(const Vector3& direction)
{
    upDirection_ = direction;
    if (controller_)
        controller_->setUpDirection(ToPxVec3(direction));
}

void Urho3DPhysX::KinematicController::SetSlopeLimit(float limit)
{
    bool isBeingEnabled = controller_ && slopeLimit_ == 0.0f && limit > 0.0f;
    slopeLimit_ = limit;
    if (controller_)
    {
        if (isBeingEnabled)
            recreatingNeeded_ = true;
        else
            controller_->setSlopeLimit(Cos<float>(slopeLimit_));
    }
}

void Urho3DPhysX::KinematicController::SetDensity(float value)
{
    if (density_ != value)
    {
        density_ = value;
        recreatingNeeded_ = true;
    }
}

void Urho3DPhysX::KinematicController::SetCapsuleHeight(float height)
{
    capsuleHeight_ = height;
    if (controller_ && controllerType_ == BOX_CONTROLLER)
    {
        auto* capsule = static_cast<PxCapsuleController*>(controller_);
        if (capsule)
            capsule->setHeight(capsuleHeight_);
    }
}

void Urho3DPhysX::KinematicController::SetCapsuleRadius(float radius)
{
    capsuleRadius_ = radius;
    if (controller_ && controllerType_ == CAPSULE_CONTROLLER)
    {
        auto* capsule = static_cast<PxCapsuleController*>(controller_);
        if (capsule)
            capsule->setRadius(capsuleRadius_);
    }
}

void Urho3DPhysX::KinematicController::SetCapsuleSize(const Vector2& size)
{
    SetCapsuleRadius(size.x_);
    SetCapsuleHeight(size.y_);
}

void Urho3DPhysX::KinematicController::SetBoxHalfHeight(float halfHeight)
{
    boxHalfHeight_ = halfHeight;
    if (controller_ && controllerType_ == BOX_CONTROLLER)
    {
        auto* box = static_cast<PxBoxController*>(controller_);
        if (box)
            box->setHalfHeight(boxHalfHeight_);
    }
}

void Urho3DPhysX::KinematicController::SetBoxHalfSideExtend(float halfExtend)
{
    boxHalfSideExtend_ = halfExtend;
    if (controller_ && controllerType_ == BOX_CONTROLLER)
    {
        auto* box = static_cast<PxBoxController*>(controller_);
        if (box)
            box->setHalfSideExtent(boxHalfSideExtend_);
    }
}

void Urho3DPhysX::KinematicController::SetBoxHalfForwadExtend(float halfExtend)
{
    boxHalfForwardExtend_ = halfExtend;
    if (controller_ && controllerType_ == BOX_CONTROLLER)
    {
        auto* box = static_cast<PxBoxController*>(controller_);
        if (box)
            box->setHalfForwardExtent(boxHalfForwardExtend_);
    }
}

void Urho3DPhysX::KinematicController::SetBoxHalfSize(const Vector3& halfSize)
{
    SetBoxHalfSideExtend(halfSize.x_);
    SetBoxHalfHeight(halfSize.y_);
    SetBoxHalfForwadExtend(halfSize.z_);
}

void Urho3DPhysX::KinematicController::SetBoxSize(const Vector3& size)
{
    SetBoxHalfSize(size * 0.5f);
}

void Urho3DPhysX::KinematicController::SetNonWalkableMode(ControllerNonWalkableMode mode)
{
    nonWalkableMode_ = mode;
    if (controller_)
        controller_->setNonWalkableMode(nonWalkableMode_ == PREV_CLIMBING ? PxControllerNonWalkableMode::ePREVENT_CLIMBING : PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING);
}

void Urho3DPhysX::KinematicController::SetMaterial(PhysXMaterial* material)
{
    if (material && material != material_)
    {
        material_ = SharedPtr<PhysXMaterial>(material);
        recreatingNeeded_ = true;
    }
}

void Urho3DPhysX::KinematicController::SetControllerType(ControllerType type)
{
    if (type != controllerType_)
    {
        controllerType_ = type;
        recreatingNeeded_ = true;
    }
}

void Urho3DPhysX::KinematicController::SetCollisionLayer(unsigned layer)
{
    collisionLayer_ = layer;
    filterData_.word0 = collisionLayer_;
    UpdateCollisionLayerAndMask();
}

void Urho3DPhysX::KinematicController::SetCollisionMask(unsigned mask)
{
    collisionMask_ = mask;
    filterData_.word1 = collisionMask_;
    UpdateCollisionLayerAndMask();
}

void Urho3DPhysX::KinematicController::UpdateCollisionLayerAndMask()
{
    filterData_.word0 = collisionLayer_;
    filterData_.word1 = collisionMask_;
    filterData_.word2 = 1; //Marker for KinematicController
    if (controller_)
    {
        PxRigidActor* actor = controller_->getActor();
        PxShape* shape(nullptr);
        actor->getShapes(&shape, 1);
        if (shape)
        {
            shape->setSimulationFilterData(filterData_);
            shape->userData = this;
        }
    }
}

void Urho3DPhysX::KinematicController::InvalidateCache()
{
    if (controller_)
        controller_->invalidateCache();
}

void Urho3DPhysX::KinematicController::Resize(float height)
{
    if (controller_)
    {
        controller_->resize(height);
        switch (controllerType_)
        {
        case Urho3DPhysX::CAPSULE_CONTROLLER:
            capsuleHeight_ = height;
            break;
        case Urho3DPhysX::BOX_CONTROLLER:
            boxHalfHeight_ = height;
            break;
        default:
            break;
        }
    }
}

void Urho3DPhysX::KinematicController::SetCCTFilterCallback(PxControllerFilterCallback* callback)
{
    filters_.mCCTFilterCallback = callback;
}

void Urho3DPhysX::KinematicController::SetControllerHitCallback(PxUserControllerHitReport* callback)
{
    if (hitCallback_ != callback)
    {
        hitCallback_ = callback;
        recreatingNeeded_ = true;
    }
}

void Urho3DPhysX::KinematicController::SetBehaviorControllerCallback(PxControllerBehaviorCallback* callback)
{
    if (behaviorCallback_ != callback)
    {
        behaviorCallback_ = callback;
        recreatingNeeded_ = true;
    }
}

void Urho3DPhysX::KinematicController::OnSceneSet(Scene* scene)
{
    if (scene)
    {
        if (scene == node_)
            URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");
        pxScene_ = scene->GetOrCreateComponent<PhysXScene>();
        hitCallback_ = pxScene_->GetControllerHitCallback();
        recreatingNeeded_ = true;
    }
}

void Urho3DPhysX::KinematicController::OnNodeSet(Node* node)
{
    if (node)
    {
        node->AddListener(this);
    }
}

void Urho3DPhysX::KinematicController::OnMarkedDirty(Node* node)
{
    UpdatePositionFromNode();
}

void Urho3DPhysX::KinematicController::OnShapeHit(const PxControllerShapeHit& hit)
{
    using namespace ShapeCollision;;
    VariantMap& eventData = GetEventDataMap();
    eventData[P_PHYSX_SCENE] = pxScene_;
    eventData[P_CONTROLLER] = this;
    WeakPtr<RigidActor> actor(static_cast<RigidActor*>(hit.actor->userData));
    eventData[P_ACTOR] = actor;
    WeakPtr<CollisionShape> shape(static_cast<CollisionShape*>(hit.shape->userData));
    eventData[P_SHAPE] = shape;
    eventData[P_POSITION] = ToVector3(hit.worldPos);
    eventData[P_NORMAL] = ToVector3(hit.worldNormal);
    eventData[P_DIR] = ToVector3(hit.dir);
    eventData[P_LENGTH] = hit.length;
    SendEvent(E_SHAPECOLLISION, eventData);
}

void Urho3DPhysX::KinematicController::OnControllerHit(const PxControllersHit& hit)
{
    using namespace ControllerCollision;
    VariantMap& eventData = GetEventDataMap();
    eventData[P_PHYSX_SCENE] = pxScene_;
    eventData[P_CONTROLLER] = this;
    SharedPtr<KinematicController> other(static_cast<KinematicController*>(hit.other->getUserData()));
    eventData[P_OTHER_CONTROLLER] = other;
    eventData[P_POSITION] = ToVector3(hit.worldPos);
    eventData[P_NORMAL] = ToVector3(hit.worldNormal);
    eventData[P_DIR] = ToVector3(hit.dir);
    eventData[P_LENGTH] = hit.length;
    //it's unsafe to call move on other controller at this point - maybe cache result and call it after move is done?
    SendEvent(E_CONTROLLERCOLLISION, eventData);
}

void Urho3DPhysX::KinematicController::UpdateNodePosition()
{
    if (controller_)
    {
        if (nodeFromFoot_)
            node_->SetWorldPosition(ToVector3(controller_->getFootPosition()));
        else
            node_->SetWorldPosition(ToVector3(controller_->getPosition()) - position_);
        lastPosition_ = node_->GetWorldPosition();
    }
}

void Urho3DPhysX::KinematicController::UpdatePositionFromNode()
{
    if (isMoving_)
        return;
    if (node_ && controller_)
    {
        Vector3 newPos = node_->GetWorldPosition();
        if (!newPos.Equals(lastPosition_))
        {
            lastPosition_ = newPos;
            controller_->setPosition(ToPxExtendedVec3(newPos + position_));
        }
    }
}

void Urho3DPhysX::KinematicController::ReleaseController()
{
    if (controller_)
    {
        controller_->release();
        controller_ = nullptr;
    }
}
