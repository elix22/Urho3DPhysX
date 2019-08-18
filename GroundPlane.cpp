#include "GroundPlane.h"
#include "StaticBody.h"
#include "CollisionShape.h"
#include <Urho3D/Core/Context.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Resource/ResourceCache.h>

Urho3DPhysX::GroundPlane::GroundPlane(Context * context) : Component(context),
body_(nullptr),
shape_(nullptr)
{
}

Urho3DPhysX::GroundPlane::~GroundPlane()
{
}

void Urho3DPhysX::GroundPlane::RegisterObject(Context * context)
{
    context->RegisterFactory<GroundPlane>("PhysX");
}

void Urho3DPhysX::GroundPlane::OnNodeSet(Node * node)
{
    if (node)
    {
        StaticModel* model = node->CreateComponent<StaticModel>();
        model->SetTemporary(true);
        model->SetModel(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Plane.mdl"));
        body_ = node->CreateComponent<StaticBody>();
        body_->SetTemporary(true);
        shape_ = node->CreateComponent<CollisionShape>();
        shape_->SetTemporary(true);
        shape_->SetShapeType(PLANE_SHAPE);
    }
}
