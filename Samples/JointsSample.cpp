#include "JointsSample.h"
#include <StaticBody.h>
#include <DynamicBody.h>
#include <CollisionShape.h>
#include <PhysXScene.h>
#include <Joint.h>
#include <SphericalJoint.h>
#include <FixedJoint.h>
#include <DistanceJoint.h>
#include <PrismaticJoint.h>
#include <RevoluteJoint.h>
#include <GroundPlane.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Core/CoreEvents.h>

JointsSample::JointsSample(Context * context) : SampleBase(context),
pxScene_(nullptr),
dbr_(nullptr)
{
}

JointsSample::~JointsSample()
{
}

void JointsSample::SampleStart()
{
    CreateSceneAndViewport();
    pxScene_ = scene_->CreateComponent<PhysXScene>();
    pxScene_->SetDebugDrawEnabled(true);
    dbr_ = scene_->CreateComponent<DebugRenderer>();
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(JointsSample, HandlePostRenderUpdate));
    Node* ground = scene_->CreateChild("Ground");
    ground->SetScale(1000.0f);
    GroundPlane* groundPlane = ground->CreateComponent<GroundPlane>();
    CreateSphericalJoints();
    CreateFixedJoints();
    CreateDistanceJoints();
    CreatePrismaticJoints();
    CreateRevoluteJoints();
}

void JointsSample::Update(float timeStep)
{
    MoveFreeCamera(timeStep);
}

void JointsSample::CreateSphericalJoints()
{
    //default joint
    Node* cube1Node = CreateCube(Vector3(-10.0f, 8.0f, -10.0f));
    StaticBody* cube1Body = cube1Node->CreateComponent<StaticBody>();
    CollisionShape* cube1Shape = cube1Node->CreateComponent<CollisionShape>();
    Node* cube2Node = CreateCube(Vector3(-10.0f, 6.0f, -10.0f));
    DynamicBody* cube2Body = cube2Node->CreateComponent<DynamicBody>();
    CollisionShape* cube2Shape = cube2Node->CreateComponent<CollisionShape>();
    Node* cube3Node = CreateCube(Vector3(-10.0f, 4.0f, -10.0f));
    DynamicBody* cube3Body = cube3Node->CreateComponent<DynamicBody>();
    CollisionShape* cube3Shape = cube3Node->CreateComponent<CollisionShape>();
    SphericalJoint* joint1 = cube1Node->CreateComponent<SphericalJoint>();
    joint1->SetOtherActor(cube2Body);
    SphericalJoint* joint2 = cube2Node->CreateComponent<SphericalJoint>();
    joint2->SetOtherActor(cube3Body);
    for (auto* j : { joint1, joint2 })
    {
        j->SetPosition(Vector3(0.0f, -0.5f, -0.0f));
        j->SetOtherPosition(Vector3(0.0f, 2.0f, 0.0f));
    }
    //limited joint
    Node* cube4Node = CreateCube(Vector3(-10.0f, 8.0f, -5.0f));
    StaticBody* cube4Body = cube4Node->CreateComponent<StaticBody>();
    CollisionShape* cube4Shape = cube4Node->CreateComponent<CollisionShape>();
    Node* cube5Node = CreateCube(Vector3(-10.0f, 6.0f, -5.0f));
    DynamicBody* cube5Body = cube5Node->CreateComponent<DynamicBody>();
    CollisionShape* cube5Shape = cube5Node->CreateComponent<CollisionShape>();
    SphericalJoint* joint3 = cube4Node->CreateComponent<SphericalJoint>();
    joint3->SetTransform(Vector3(0.0f, -0.5f, 0.0f), Quaternion(-90.0f, Vector3::FORWARD));
    joint3->SetOtherPosition(Vector3(0.0f, 2.0f, 0.0f));
    joint3->SetLimitEnabled(true);
    joint3->SetUseSoftLimit(false);
    joint3->SetContactDistance(0.1f);
    joint3->SetStiffness(100.0f);
    joint3->SetLimitConeAngles(Vector2(1.2f, 1.2f));
    joint3->SetOtherActor(cube5Body);
}

void JointsSample::CreateFixedJoints()
{
    Vector3 basePos(-5.0f, 1.0f, -10.0f);
    Node* firstNode = CreateCube(basePos);
    DynamicBody* firstBody = firstNode->CreateComponent<DynamicBody>();
    CollisionShape* firstShape = firstNode->CreateComponent<CollisionShape>();

    for (unsigned i = 0; i < 10; ++i)
    {
        Node* secondNode = CreateCube(basePos + Vector3(0.0f, 1.0f + (float)i, 0.0f));
        DynamicBody* secondBody = secondNode->CreateComponent<DynamicBody>();
        CollisionShape* secondShape = secondNode->CreateComponent<CollisionShape>();
        FixedJoint* joint = firstNode->CreateComponent<FixedJoint>();
        joint->SetTransform(Vector3(0.0f, 0.5f, 0.0f), Quaternion(15.0f, Vector3::UP));
        joint->SetOtherPosition(Vector3(0.0f, -0.5f, 0.0f));
        joint->SetOtherActor(secondBody);
        firstNode = secondNode;
        firstBody = secondBody;
    }
}

void JointsSample::CreateDistanceJoints()
{
    Vector3 basePos(-5.0f, 40.0f, -5.0f);
    Node* firstNode = CreateCube(basePos);
    RigidActor* firstBody = firstNode->CreateComponent<StaticBody>();
    CollisionShape* firstShape = firstNode->CreateComponent<CollisionShape>();

    for (unsigned i = 0; i < 10; ++i)
    {
        Node* secondNode = CreateCube(basePos - Vector3(0.0f, 1.0f + (float)i, 0.0f));
        DynamicBody* secondBody = secondNode->CreateComponent<DynamicBody>();
        CollisionShape* secondShape = secondNode->CreateComponent<CollisionShape>();
        DistanceJoint* joint = firstNode->CreateComponent<DistanceJoint>();
        joint->SetStiffness(35.0f);
        joint->SetSpringEnabled(true);
        joint->SetMinDistance(0.25f);
        joint->SetMinDistanceEnabled(true);
        joint->SetMaxDistance(1.0f);
        joint->SetMaxDistanceEnabled(true);
        joint->SetTransform(Vector3(0.0f, -0.5f, 0.0f), Quaternion(15.0f, Vector3::UP));
        joint->SetOtherPosition(Vector3(0.0f, 0.75f, 0.0f));
        joint->SetOtherActor(secondBody);
        firstNode = secondNode;
        firstBody = secondBody;
    }
}

void JointsSample::CreatePrismaticJoints()
{
    Node* staticNode = CreateCube(Vector3::UP);
    StaticBody* staticBody = staticNode->CreateComponent<StaticBody>();
    CollisionShape* staticShape = staticNode->CreateComponent<CollisionShape>();

    Node* cube1Node = CreateCube(Vector3(-1.0f, 1.0f, 0.0f));
    DynamicBody* cube1Body = cube1Node->CreateComponent<DynamicBody>();
    CollisionShape* cube1Shape = cube1Node->CreateComponent<CollisionShape>();
    PrismaticJoint* joint1 = staticNode->CreateComponent<PrismaticJoint>();
    joint1->SetTransform(Vector3(-1.01f, 0.0f, 0.0f), Quaternion(90, Vector3::UP));
    joint1->SetOtherPosition(Vector3(0.52f, 0.0f, 0.0f));
    joint1->SetUseSoftLimit(false);
    joint1->SetOtherActor(cube1Body);

    Node* cube2Node = CreateCube(Vector3(1.0f, 1.0f, 0.0f));
    DynamicBody* cube2Body = cube2Node->CreateComponent<DynamicBody>();
    CollisionShape* cube2Shape = cube2Node->CreateComponent<CollisionShape>();
    PrismaticJoint* joint2 = staticNode->CreateComponent<PrismaticJoint>();
    joint2->SetTransform(Vector3(1.01f, 0.0f, 0.0f), Quaternion(-90, Vector3::UP));
    joint2->SetPosition(Vector3(1.1f, 0.0f, 0.0f));
    joint2->SetStiffness(100.0f);
    joint2->SetDamping(1.0f);
    joint2->SetUseSoftLimit(true);
    joint2->SetOtherActor(cube2Body);
    
    for (auto* j : { joint1, joint2 })
    {
        j->SetLimitEnabled(true);
        j->SetLowerLimit(-5.0f);
        j->SetUpperLimit(5.0f);
    }
}

void JointsSample::CreateRevoluteJoints()
{
    Node* static1Node = CreateCube(Vector3(5.0f, 3.0f, 0.0f));
    StaticBody* static1Body = static1Node->CreateComponent<StaticBody>();
    CollisionShape* statc1Shape = static1Node->CreateComponent<CollisionShape>();

    Node* dynamic1Node = CreateCube(Vector3(6.0f, 3.0f, -5.0f));
    DynamicBody* dynamic1Body = dynamic1Node->CreateComponent<DynamicBody>();
    CollisionShape* dynamic1Shape = dynamic1Node->CreateComponent<CollisionShape>();
    RevoluteJoint* joint1 = static1Node->CreateComponent<RevoluteJoint>();
    joint1->SetOtherTransform(Vector3(0.0f, 1.75f, 0.0f));
    joint1->SetOtherActor(dynamic1Body);

    Node* static2Node = CreateCube(Vector3(5.0f, 3.0f, -5.0f));
    StaticBody* static2Body = static2Node->CreateComponent<StaticBody>();
    CollisionShape* statc2Shape = static2Node->CreateComponent<CollisionShape>();

    Node* dynamic2Node = CreateCube(Vector3(6.0f, 3.0f, -5.0f));
    DynamicBody* dynamic2Body = dynamic2Node->CreateComponent<DynamicBody>();
    CollisionShape* dynamic2Shape = dynamic2Node->CreateComponent<CollisionShape>();
    RevoluteJoint* joint2 = static2Node->CreateComponent<RevoluteJoint>();
    joint2->SetOtherTransform(Vector3(0.0f, 1.75f, 0.0f));
    joint2->SetOtherActor(dynamic2Body);
    joint2->SetDriveEnabled(true);
    joint2->SetDriveVelocity(6.0f);
    joint2->SetDriveGearRatio(2.0f);
}

void JointsSample::HandlePostRenderUpdate(StringHash eventType, VariantMap & eventData)
{
    if (dbr_ && pxScene_)
        pxScene_->DrawDebugGeometry(dbr_);
}
