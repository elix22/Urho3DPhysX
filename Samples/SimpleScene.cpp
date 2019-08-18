#include "SimpleScene.h"
#include <PhysXScene.h>
#include <DynamicBody.h>
#include <StaticBody.h>
#include <GroundPlane.h>
#include <CollisionShape.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Material.h>

SimpleScene::SimpleScene(Context * context) : SampleBase(context)
{
}

SimpleScene::~SimpleScene()
{
}

void SimpleScene::SampleStart()
{
    CreateSceneAndViewport();
    using namespace Urho3DPhysX;
    PhysXScene* pxScene = scene_->CreateComponent<PhysXScene>();
    Node* groundNode = scene_->CreateChild("Ground");
    groundNode->SetScale(1000.0f);
    groundNode->CreateComponent<GroundPlane>();    
    //create pyramid of dynamic boxes
    for (int y = 0; y < 10; ++y)
    {
        for (int x = -y; x <= y; ++x)
        {
            Node* n = CreateCube(Vector3((float)x, -(float)y + 10.0f, 0.0f));
            DynamicBody* body = n->CreateComponent<DynamicBody>();
            CollisionShape* shape = n->CreateComponent<CollisionShape>();
        }
    }
}

void SimpleScene::Update(float timeStep)
{
    MoveFreeCamera(timeStep);
}
