#include "UrhoPhysXSamples.h"
#include "SimpleScene.h"
#include "JointsSample.h"
#include "EventsSample.h"
#include "RaycastSample.h"
#include "MaterialsSample.h"
#include <Physics.h>
#include <Urho3D/Engine/EngineDefs.h>
#include <Urho3D/Resource/XMLFile.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Engine/DebugHud.h>
#include <Urho3D/Engine/Console.h>
#include <Urho3D/UI/Button.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include <Urho3D/UI/UIEvents.h>
#include <Urho3D/Input/Input.h>

using namespace Urho3DPhysX;
PhysXSamples::PhysXSamples(Context * context) : Application(context),
currentSample_(nullptr),
samplesMenu_(nullptr)
{
}

PhysXSamples::~PhysXSamples()
{
}

void PhysXSamples::Setup()
{
#ifdef _DEBUG
    engineParameters_[EP_FULL_SCREEN] = false;
#endif // _DEBUG
}

void PhysXSamples::Start()
{
    GetSubsystem<ResourceCache>()->AddResourceDir("PhysXData");
    XMLFile* defStyle = GetSubsystem<ResourceCache>()->GetResource<XMLFile>("UI/DefaultStyle.xml");
    DebugHud* dbh = GetSubsystem<Engine>()->CreateDebugHud();
    dbh->SetDefaultStyle(defStyle);
    Console* c = GetSubsystem<Engine>()->CreateConsole();
    c->SetDefaultStyle(defStyle);
    RegisterPhysXLibrary(context_);
    context_->RegisterSubsystem<Physics>();
    Physics* physics = GetSubsystem<Physics>();
    physics->SetDefBraodPhaseType(PxBroadPhaseType::eABP);
    physics->InitializePhysX();
    CreateSamplesMenu();
    SubscribeToEvent(E_KEYUP, URHO3D_HANDLER(PhysXSamples, HandleKeyUp));
}

void PhysXSamples::CreateSamplesMenu()
{
    auto* ui = GetSubsystem<UI>();
    auto* cache = GetSubsystem<ResourceCache>();
    samplesMenu_ = ui->LoadLayout(cache->GetResource<XMLFile>("UI/SamplesMenu.xml"));
    if (samplesMenu_)
    {
        ui->GetRoot()->AddChild(samplesMenu_);
        GetSubsystem<Input>()->SetMouseVisible(true);
        PODVector<UIElement*> children;
        samplesMenu_->GetChildren(children, true);
        for (auto* child : children)
        {
            Button* b = child->Cast<Button>();
            if (b)
                SubscribeToEvent(b, E_RELEASED, URHO3D_HANDLER(PhysXSamples, HandleSamplesMenuButton));
        }
    }
}

void PhysXSamples::HandleSamplesMenuButton(StringHash eventType, VariantMap & eventData)
{
    if (currentSample_)
    {
        currentSample_->SampleEnd();
        currentSample_.Reset();
    }
    using namespace Released;
    Button* btn = static_cast<Button*>(eventData[P_ELEMENT].GetPtr());
    if (btn)
    {
        const String& name = btn->GetName();
        if (name == "Sample1")
        {
            currentSample_ = SharedPtr<SampleBase>(new SimpleScene(context_));
        }
        else if (name == "Sample2")
        {
            currentSample_ = SharedPtr<SampleBase>(new MaterialsSample(context_));
        }
        else if (name == "Sample3")
        {
            currentSample_ = SharedPtr<SampleBase>(new JointsSample(context_));
        }
        else if (name == "Sample4")
        {
            currentSample_ = SharedPtr<SampleBase>(new RaycastSample(context_));
        }
        if (currentSample_)
        {
            currentSample_->SampleStart();
            ToggleSamplesMenu();
        }
    }
}

void PhysXSamples::HandleKeyUp(StringHash eventType, VariantMap & eventData)
{
    using namespace KeyUp;
    unsigned key = eventData[P_KEY].GetUInt();
    if (key == KEY_ESCAPE)
    {
        ToggleSamplesMenu();
    }
}

void PhysXSamples::ToggleSamplesMenu()
{
    if (samplesMenu_)
    {
        bool isEnabled = samplesMenu_->IsVisible();
        if (isEnabled)
        {
            samplesMenu_->SetDeepEnabled(false);
            samplesMenu_->SetVisible(false);
        }
        else
        {
            samplesMenu_->SetVisible(true);
            samplesMenu_->ResetDeepEnabled();
        }
        GetSubsystem<Input>()->SetMouseVisible(!isEnabled);
    }
}
