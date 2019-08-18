#include "StaticBody.h"
#include "Physics.h"
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Context.h>

Urho3DPhysX::StaticBody::StaticBody(Context * context) : RigidActor(context)
{
    auto* physics = GetSubsystem<Physics>();
    if (physics)
    {
        auto* px = physics->GetPhysics();
        if (px)
        {
            actor_ = px->createRigidStatic(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
            actor_->userData = this;
        }
    }
    else
    {
        URHO3D_LOGERROR("Physics subsystem must be registred before creating physx objects.");
    }
}

Urho3DPhysX::StaticBody::~StaticBody()
{
}

void Urho3DPhysX::StaticBody::RegisterObject(Context * context)
{
    context->RegisterFactory<StaticBody>("PhysX");
}
