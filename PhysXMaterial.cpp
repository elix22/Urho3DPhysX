#include "PhysXMaterial.h"
#include "Physics.h"
#include <Urho3D/Resource/XMLFile.h>
#include <Urho3D/Resource/JSONFile.h>
#include <Urho3D/IO/Serializer.h>
#include <Urho3D/IO/Deserializer.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Core/Context.h>

static const float DEF_DYNAMIC_FRICTION = 1.0f;
static const float DEF_STATIC_FRICTION = 1.0f;
static const float DEF_MAT_RESTITUTION = 0.3f;

Urho3DPhysX::PhysXMaterial::PhysXMaterial(Context * context) : Resource(context),
xmlLoadFile_(nullptr),
dynamicFriction_(DEF_DYNAMIC_FRICTION),
staticFriction_(DEF_STATIC_FRICTION),
restitution_(DEF_MAT_RESTITUTION)
{
    auto* physics = GetSubsystem<Physics>();
    auto* px = physics->GetPhysics();
    material_ = px->createMaterial(dynamicFriction_, staticFriction_, restitution_);
    material_->userData = this;
}

Urho3DPhysX::PhysXMaterial::~PhysXMaterial()
{
    if (material_)
        material_->release();
}

void Urho3DPhysX::PhysXMaterial::RegisterObject(Context * context)
{
    context->RegisterFactory<PhysXMaterial>();
}

bool Urho3DPhysX::PhysXMaterial::BeginLoad(Deserializer & source)
{
    //if physics is not registered return
    auto* physics = GetSubsystem<Physics>();
    if (!physics)
        return false;

    String extension = GetExtension(source.GetName());
    bool success = false;
    if (extension == ".xml")
    {
        return BeginLoadXML(source);
    }
    return false;
}

bool Urho3DPhysX::PhysXMaterial::EndLoad()
{
    bool success = false;
    if (xmlLoadFile_)
        success = Load(xmlLoadFile_->GetRoot("PxMaterial"));
    xmlLoadFile_ = nullptr;
    return success;
}

bool Urho3DPhysX::PhysXMaterial::Load(const XMLElement & source)
{
    if (source.IsNull())
    {
        URHO3D_LOGERROR("Failed to load PhysXMaterial - source is null.");
        return false;
    }
    if (material_)
    {
        XMLElement dynamicFrictionEl = source.GetChild("dynamicFriction");
        if (dynamicFrictionEl)
            SetDynamicFriction(dynamicFrictionEl.GetFloat("value"));
        XMLElement staticFrictionEl = source.GetChild("staticFriction");
        if (staticFrictionEl)
            SetStaticFriction(staticFrictionEl.GetFloat("value"));
        XMLElement restitutionEl = source.GetChild("restitution");
        if (restitutionEl)
            SetRestitution(restitutionEl.GetFloat("value"));
        return true;
    }
    return false;
}

bool Urho3DPhysX::PhysXMaterial::Save(Serializer & dest) const
{
    return false;
}

void Urho3DPhysX::PhysXMaterial::SetDynamicFriction(float val)
{
    dynamicFriction_ = Clamp(val, 0.0f, PX_MAX_F32);
    if (material_)
        material_->setDynamicFriction(dynamicFriction_);
}

void Urho3DPhysX::PhysXMaterial::SetStaticFriction(float val)
{
    staticFriction_ = Clamp(val, 0.0f, PX_MAX_F32);
    if (material_)
        material_->setStaticFriction(staticFriction_);
}

void Urho3DPhysX::PhysXMaterial::SetRestitution(float val)
{
    restitution_ = Clamp(val, 0.0f, 1.0f);
    if (material_)
        material_->setRestitution(restitution_);
}

void Urho3DPhysX::PhysXMaterial::ResetToDefault()
{
    SetDynamicFriction(DEF_DYNAMIC_FRICTION);
    SetStaticFriction(DEF_STATIC_FRICTION);
    SetRestitution(DEF_MAT_RESTITUTION);
}

bool Urho3DPhysX::PhysXMaterial::BeginLoadXML(Deserializer & source)
{
    ResetToDefault();
    xmlLoadFile_ = SharedPtr<XMLFile>(new XMLFile(context_));
    if (xmlLoadFile_->Load(source))
    {
        return true;
    }
    xmlLoadFile_ = nullptr;
    return false;
}
