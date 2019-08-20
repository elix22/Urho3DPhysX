#include "StressTest.h"
#include <PhysXScene.h>
#include <DynamicBody.h>
#include <StaticBody.h>
#include <GroundPlane.h>
#include <CollisionShape.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Zone.h>

using namespace Urho3DPhysX;
StressTest::StressTest(Context * context) : SampleBase(context)
{
}

StressTest::~StressTest()
{
}

void StressTest::SampleStart()
{
    //This sample recreates urho's physics stress test
    CreateSceneAndViewport();
    zone_->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
    zone_->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
    zone_->SetFogColor(Color(0.5f, 0.5f, 0.7f));
    zone_->SetFogStart(100.0f);
    zone_->SetFogEnd(300.0f);
    //Use scene's vars to set GPU broad phase algorithm for this sample (ABP is default for samples application).
    scene_->SetVar("BPT", "GPU");
    PhysXScene* pxScene = scene_->CreateComponent<PhysXScene>();
    //Disable collision events processing for this sample
    pxScene->SetProcessSimulationEvents(false);
    {
        //Create floor, like original sample use scaled box, do not use ground plane since it's not supported by GPU
        Node* floorNode = scene_->CreateChild("Floor");
        floorNode->SetPosition(Vector3(0.0f, -0.5f, 0.0f));
        floorNode->SetScale(Vector3(500.0f, 1.0f, 500.0f));
        auto* floorObject = floorNode->CreateComponent<StaticModel>();
        floorObject->SetModel(cache_->GetResource<Model>("Models/Box.mdl"));
        floorObject->SetMaterial(cache_->GetResource<Material>("Materials/StoneTiled.xml"));

        floorNode->CreateComponent<StaticBody>();
        //No need to call collisionShape->SetBox, since this is a default shape
        floorNode->CreateComponent<CollisionShape>();
    }
    {
        // Create static mushrooms with triangle mesh collision
        const unsigned NUM_MUSHROOMS = 50;
        for (unsigned i = 0; i < NUM_MUSHROOMS; ++i)
        {
            Node* mushroomNode = scene_->CreateChild("Mushroom");
            mushroomNode->SetPosition(Vector3(Random(400.0f) - 200.0f, 0.0f, Random(400.0f) - 200.0f));
            mushroomNode->SetRotation(Quaternion(0.0f, Random(360.0f), 0.0f));
            mushroomNode->SetScale(5.0f + Random(5.0f));
            auto* mushroomObject = mushroomNode->CreateComponent<StaticModel>();
            mushroomObject->SetModel(cache_->GetResource<Model>("Models/Mushroom.mdl"));
            mushroomObject->SetMaterial(cache_->GetResource<Material>("Materials/Mushroom.xml"));
            mushroomObject->SetCastShadows(true);

            mushroomNode->CreateComponent<StaticBody>();
            auto* shape = mushroomNode->CreateComponent<CollisionShape>();
            //This will use model from static/animated model component attached to the same node as collision shape.
            shape->SetShapeType(TRIANGLEMESH_SHAPE);
            //It's possible to use custom model by calling CollisionShape::SetCustomModel
        }
        {
            // Create a large amount of falling physics objects
#ifndef _DEBUG
            const unsigned NUM_OBJECTS = 10000;
#else
            const unsigned NUM_OBJECTS = 1000;
#endif // !_DEBUG
            for (unsigned i = 0; i < NUM_OBJECTS; ++i)
            {
                Node* boxNode = scene_->CreateChild("Box");
                boxNode->SetPosition(Vector3(0.0f, i * 2.0f + 100.0f, 0.0f));
                auto* boxObject = boxNode->CreateComponent<StaticModel>();
                boxObject->SetModel(cache_->GetResource<Model>("Models/Box.mdl"));
                boxObject->SetMaterial(cache_->GetResource<Material>("Materials/StoneSmall.xml"));
                boxObject->SetCastShadows(true);

                // Give the RigidBody mass to make it movable and also adjust friction
                boxNode->CreateComponent<DynamicBody>();
                boxNode->CreateComponent<CollisionShape>();
            }
        }
    }
    
}

void StressTest::Update(float timeStep)
{
    MoveFreeCamera(timeStep);
}
