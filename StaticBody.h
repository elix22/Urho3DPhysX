#pragma once
#include "RigidActor.h"

namespace Urho3DPhysX
{
    class URHOPX_API StaticBody : public RigidActor
    {
        URHO3D_OBJECT(StaticBody, RigidActor);

    public:
        StaticBody(Context* context);
        ~StaticBody();

        static void RegisterObject(Context* context);
    };
}
