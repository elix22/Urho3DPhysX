#pragma once
#include <Urho3D/Core/Object.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Resource/ResourceCache.h>

namespace Urho3D
{
    class Zone;
    class Camera;
    class Text;
}
using namespace Urho3D;

class SampleBase : public Object
{
    URHO3D_OBJECT(SampleBase, Object);

public:
    SampleBase(Context* context);
    ~SampleBase();

    virtual void SampleStart() {};
    virtual void SampleEnd();
    virtual void CreateScene();
    virtual void CreateSceneAndViewport();

    //drop cubes
    void DropCubes(const Vector3& position = Vector3(0.0f, 50.0f, 0.0f));
    //multi-drop cubes
    void MultiDropCubes(const Vector3& offset = Vector3::ZERO);
    //shoot sphere
    void ShootSphere();
    //create single cube
    Node* CreateCube(const Vector3& pos, const Quaternion& rotation = Quaternion::IDENTITY, Node* parent = nullptr);

protected:
    virtual void Update(float timeStep) {};
    virtual void OnKeyUp(Key key);
    virtual void OnMouseBtnUp(MouseButton btn);
    virtual void CreateInstructions();
    void ToggleInstructions();

    void MoveFreeCamera(float timeStep);
    ResourceCache* cache_;
    SharedPtr<Scene> scene_;
    Zone* zone_;
    Node* cameraNode_;
    Camera* camera_;
    Text* instructionsText_;
    String instructions_;

    float yaw_{};
    float pitch_{};

    bool updateEnabled_;
private:
    void HandleUpdate(StringHash eventType, VariantMap& eventData);
    void HandleKeyUp(StringHash eventType, VariantMap& eventData);
    void HandleMouseBtnUp(StringHash eventType, VariantMap& eventData);
};
