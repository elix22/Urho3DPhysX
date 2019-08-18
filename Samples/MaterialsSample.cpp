#include "MaterialsSample.h"
#include <StaticBody.h>
#include <DynamicBody.h>
#include <GroundPlane.h>
#include <CollisionShape.h>
#include <PhysXMaterial.h>
#include <PhysXScene.h>
#include <Urho3D/UI/Text3D.h>
#include <Urho3D/UI/Font.h>

MaterialsSample::MaterialsSample(Context * context) : SampleBase(context)
{
}

MaterialsSample::~MaterialsSample()
{
}

void MaterialsSample::SampleStart()
{
    CreateSceneAndViewport();
    PhysXScene* pxScene = scene_->CreateComponent<PhysXScene>();
    Node* groundNode = scene_->CreateChild("Ground");
    groundNode->SetScale(1000.0f);
    groundNode->CreateComponent<GroundPlane>();

    CreateSlide(Vector3(-10.0f, 2.0f, 0.0f), String::EMPTY);
    CreateSlide(Vector3(-6.0f, 2.0f, 0.0f), "PxMaterials/Material1.xml", true);
    CreateSlide(Vector3(-2.0f, 2.0f, 0.0f), "PxMaterials/Material2.xml", true);
    CreateSlide(Vector3(2.0f, 2.0f, 0.0f), "PxMaterials/Material3.xml", true);
}

void MaterialsSample::Update(float timeStep)
{
    MoveFreeCamera(timeStep);
}

void MaterialsSample::CreateSlide(const Vector3 & position, const String & matName, bool applyToCube)
{
    Node* slideNode = CreateCube(position, Quaternion(35, Vector3::LEFT));
    slideNode->SetScale(Vector3(3.0f, 1.0f, 10.0f));
    StaticBody* slideBody = slideNode->CreateComponent<StaticBody>();
    CollisionShape* slideShape = slideNode->CreateComponent<CollisionShape>();

    Node* textNode = slideNode->CreateChild("Text");
    textNode->SetPosition(Vector3(0.0f, 0.51f, 0.0f));
    textNode->SetRotation(Quaternion(-90.0f, Vector3::LEFT));
    auto* matText = textNode->CreateComponent<Text3D>();
    matText->SetText("Default material");
    matText->SetFont(cache_->GetResource<Font>("Fonts/BlueHighway.sdf"));
    matText->SetHorizontalAlignment(HA_CENTER);
    textNode->SetScale(0.2f);
    matText->SetColor(Color::GREEN);

    SharedPtr<PhysXMaterial> material(cache_->GetResource<PhysXMaterial>(matName));
    if (material)
    {
        matText->SetText(material->GetName());
        slideShape->SetMaterial(material);
    }

    for (unsigned i = 0; i < 5; ++i)
    {
        Node* cubeNode = CreateCube(position + Vector3(0.0f, 5.51f + i*3, 3.5f), Quaternion(35, Vector3::LEFT));
        DynamicBody* cubeBody = cubeNode->CreateComponent<DynamicBody>();
        CollisionShape* cubeShape = cubeNode->CreateComponent<CollisionShape>();
        if (applyToCube)
            cubeShape->SetMaterial(material);
    }  
}
