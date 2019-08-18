#include <Urho3D/Engine/Application.h>

namespace Urho3D
{
    class UIElement;
}
using namespace Urho3D;

class SampleBase;

class PhysXSamples : public Application
{
public:
    PhysXSamples(Context* context);
    ~PhysXSamples();

    void Setup() override;
    void Start() override;

private:
    void CreateSamplesMenu();
    void ToggleSamplesMenu();
    void HandleSamplesMenuButton(StringHash eventType, VariantMap& eventData);
    void HandleKeyUp(StringHash eventType, VariantMap& eventData);
    SharedPtr<SampleBase> currentSample_;
    SharedPtr<UIElement> samplesMenu_;
};

URHO3D_DEFINE_APPLICATION_MAIN(PhysXSamples);