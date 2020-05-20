#include "SampleBase.h"
#include <DynamicBody.h>
#include <StaticBody.h>
#include <CollisionShape.h>
#include <PhysXMaterial.h>
#include <PhysXEvents.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Graphics/Viewport.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Input/InputEvents.h>
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/DebugHud.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Engine/Console.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/Scene/SceneEvents.h>

using namespace Urho3DPhysX;

SampleBase::SampleBase(Context * context) : Object(context),
scene_(nullptr),
zone_(nullptr),
cameraNode_(nullptr),
camera_(nullptr),
updateEnabled_(true)
{
    instructions_ = "Press TAB to toggle instructions.\n"
        "Press ESC to toggle samples menu.\n"
        "Use WSAD keys and mouse to move.\n"
        "Press SPACE to shoot a sphere.\n"
        "Press 1 or 2 to drop cubes.";
    cache_ = GetSubsystem<ResourceCache>();
    CreateInstructions();
    SubscribeToEvent(E_KEYUP, URHO3D_HANDLER(SampleBase, HandleKeyUp));
    SubscribeToEvent(E_MOUSEBUTTONUP, URHO3D_HANDLER(SampleBase, HandleMouseBtnUp));
}

SampleBase::~SampleBase()
{
}

void SampleBase::SampleEnd()
{
    if (instructionsText_)
    {
        instructionsText_->Remove();
    }
}

void SampleBase::CreateScene()
{
    scene_ = SharedPtr<Scene>(new Scene(context_));
    Octree* o = scene_->CreateComponent<Octree>();
    zone_ = scene_->CreateComponent<Zone>();
    zone_->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
    zone_->SetFogColor(Color::BLACK);
    Node* lightNode = scene_->CreateChild("Light");
    lightNode->SetDirection(Vector3(0.6f, -1.0f, 0.8f));
    Light* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetCastShadows(true); 
    light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    cameraNode_ = scene_->CreateChild("CameraNode");
    cameraNode_->SetWorldPosition(Vector3(0.0f, 15.0f, -10.0f));
    cameraNode_->LookAt(Vector3::ZERO);
    camera_ = cameraNode_->CreateComponent<Camera>();
    camera_->SetFarClip(300.0f);
    SubscribeToEvent(scene_, E_SCENEUPDATE, URHO3D_HANDLER(SampleBase, HandleUpdate));
    SubscribeToEvent(scene_, E_PX_PRESIMULATION, URHO3D_HANDLER(SampleBase, HandleFixedUpdate));
}

void SampleBase::CreateSceneAndViewport()
{
    if (scene_)
    {
        //TODO cleanup
    }
    CreateScene();
    SharedPtr<Viewport> viewport = SharedPtr<Viewport>(new Viewport(context_, scene_, camera_));
    GetSubsystem<Renderer>()->SetViewport(0, viewport);
}

void SampleBase::DropCubes(const Vector3& position)
{
    for (unsigned i = 0; i < 25; ++i)
    {
        Node* n = CreateCube(Vector3(position.x_, position.y_ + i * 2, position.z_));
        DynamicBody* body = n->CreateComponent<DynamicBody>();
        CollisionShape* shape = n->CreateComponent<CollisionShape>();
        shape->SetBox(Vector3::ONE);
        body->SetSleepThreshold(0.1f);
    }
}

void SampleBase::MultiDropCubes(const Vector3 & offset)
{
    DropCubes(Vector3(10.0f, 50.0f, 10.0f) + offset);
    DropCubes(Vector3(10.0f, 50.0f, -10.0f) + offset);
    DropCubes(Vector3(-10.0f, 50.0f, 10.0f) + offset);
    DropCubes(Vector3(-10.0f, 50.0f, -10.0f) + offset);
}

void SampleBase::ShootSphere()
{
    Node* n = scene_->CreateChild("Sphere");
    n->SetWorldPosition(cameraNode_->GetWorldPosition());
    n->SetWorldRotation(cameraNode_->GetWorldRotation());
    StaticModel* m = n->CreateComponent<StaticModel>();
    m->SetModel(cache_->GetResource<Model>("Models/Sphere.mdl"));
    m->SetCastShadows(true);
    CollisionShape* shape = n->CreateComponent<CollisionShape>();
    shape->SetSphere();
    DynamicBody* body = n->CreateComponent<DynamicBody>();
    body->ApplyImpulse(cameraNode_->GetWorldDirection() * 30.0f);
    body->SetCCDEnabled(true);
}

Node* SampleBase::CreateCube(const Vector3 & pos, const Quaternion& rotation, Node* parent)
{
    if (scene_)
    {
        if (!parent)
            parent = scene_;
        Node* cubeNode = parent->CreateChild("Cube");
        cubeNode->SetPosition(pos);
        cubeNode->SetRotation(rotation);
        StaticModel* m = cubeNode->CreateComponent<StaticModel>();
        m->SetModel(cache_->GetResource<Model>("Models/Box.mdl"));
        m->SetCastShadows(true);
        return cubeNode;
    }
    return nullptr;
}

void SampleBase::OnKeyUp(Key key)
{
    switch (key)
    {
    case(KEY_F1):
        GetSubsystem<Console>()->Toggle();
        break;
    case(KEY_F2):
        GetSubsystem<DebugHud>()->Toggle(DEBUGHUD_SHOW_PROFILER);
        break;
    case(KEY_1):
        DropCubes();
        break;
    case(KEY_2):
        MultiDropCubes();
        break;
    case(KEY_SPACE):
        ShootSphere();
        break;
    case(KEY_TAB):
        ToggleInstructions();
        break;
    default:
        break;
    }
}

void SampleBase::OnMouseBtnUp(MouseButton btn)
{
}

void SampleBase::CreateInstructions()
{
    auto* ui = GetSubsystem<UI>();
    instructionsText_ = ui->GetRoot()->CreateChild<Text>();
    instructionsText_->SetText(instructions_);
    instructionsText_->SetFont(cache_->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);

    instructionsText_->SetTextAlignment(HA_CENTER);
    instructionsText_->SetHorizontalAlignment(HA_CENTER);
    instructionsText_->SetVerticalAlignment(VA_CENTER);
    instructionsText_->SetPosition(0, ui->GetRoot()->GetHeight() / 4);
}

void SampleBase::ToggleInstructions()
{
    if (instructionsText_)
    {
        instructionsText_->SetVisible(!instructionsText_->IsVisible());
    }
}

void SampleBase::MoveFreeCamera(float timeStep)
{
    if (cameraNode_)
    {
        auto* input = GetSubsystem<Input>();
        if (input->IsMouseVisible())
            return;
        const float moveSpeed = 20.0f;
        const float mouseSens = 0.1f;
        IntVector2 mouseMove = input->GetMouseMove();
        yaw_ += mouseSens * mouseMove.x_;
        pitch_ += mouseSens * mouseMove.y_;
        cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
        pitch_ = Clamp(pitch_, -90.0f, 90.0f);
        if (input->GetKeyDown(KEY_W))
            cameraNode_->Translate(Vector3::FORWARD * moveSpeed * timeStep);
        if (input->GetKeyDown(KEY_S))
            cameraNode_->Translate(Vector3::BACK * moveSpeed * timeStep);
        if (input->GetKeyDown(KEY_A))
            cameraNode_->Translate(Vector3::LEFT * moveSpeed * timeStep);
        if (input->GetKeyDown(KEY_D))
            cameraNode_->Translate(Vector3::RIGHT * moveSpeed * timeStep);
    }
}

void SampleBase::HandleUpdate(StringHash eventType, VariantMap & eventData)
{
    if (updateEnabled_)
        Update(eventData[Update::P_TIMESTEP].GetFloat());
}

void SampleBase::HandleFixedUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace PhysXPreSimulation;
    FixedUpdate(eventData[PhysXPreSimulation::P_TIMESTEP].GetFloat());
}

void SampleBase::HandleKeyUp(StringHash eventType, VariantMap & eventData)
{
    using namespace KeyUp;
    unsigned key = eventData[P_KEY].GetUInt();
    OnKeyUp(static_cast<Key>(key));
}

void SampleBase::HandleMouseBtnUp(StringHash eventType, VariantMap & eventData)
{
    using namespace MouseButtonUp;
    unsigned btn = eventData[P_BUTTON].GetUInt();
    OnMouseBtnUp(static_cast<MouseButton>(btn));
}