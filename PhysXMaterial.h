#pragma once
#include "PhysXUtils.h"
#include <PxMaterial.h>
#include <Urho3D/Resource/Resource.h>

namespace Urho3D
{
    class XMLFile;
}
using namespace Urho3D;
using namespace physx;

namespace Urho3DPhysX
{
    class URHOPX_API PhysXMaterial : public Resource
    {
        URHO3D_OBJECT(PhysXMaterial, Resource);

    public:
        PhysXMaterial(Context* context);
        ~PhysXMaterial();

        static void RegisterObject(Context* context);

        bool BeginLoad(Deserializer& source) override;
        bool EndLoad() override;
        bool Load(const XMLElement& source);
        bool Save(Serializer& dest) const override;

        float GetDynamicFriction() const { return dynamicFriction_; }
        void SetDynamicFriction(float val);
        float GetStaticFriction() const { return staticFriction_; }
        void SetStaticFriction(float val);
        float GetRestitution() const { return restitution_; }
        void SetRestitution(float val);

        PxMaterial* GetMaterial() { return material_; }

    private:
        void ResetToDefault();

        float dynamicFriction_;
        float staticFriction_;
        float restitution_;

        bool BeginLoadXML(Deserializer& source);
        SharedPtr<XMLFile> xmlLoadFile_;
        PxMaterial* material_;
    };
}
