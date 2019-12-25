#pragma once
#include "PhysXUtils.h"
#include <Urho3D/Scene/Component.h>

using namespace Urho3D;

namespace Urho3DPhysX
{
    class StaticBody;
    class CollisionShape;

    class URHOPX_API GroundPlane : public Component
    {
        URHO3D_OBJECT(GroundPlane, Component);

    public:
        GroundPlane(Context* context);
        ~GroundPlane();

        static void RegisterObject(Context* context);

        void OnNodeSet(Node* node) override;

    private:
        WeakPtr<StaticBody> body_;
        WeakPtr<CollisionShape> shape_;
    };
}
