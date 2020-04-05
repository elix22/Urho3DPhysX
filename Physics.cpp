#include "Physics.h"
#include "PhysXEvents.h"
#include "PhysXScene.h"
#include "StaticBody.h"
#include "DynamicBody.h"
#include "PhysXMaterial.h"
#include "CollisionShape.h"
#include "SphericalJoint.h"
#include "DistanceJoint.h"
#include "FixedJoint.h"
#include "PrismaticJoint.h"
#include "RevoluteJoint.h"
#include "GroundPlane.h"
#include "KinematicController.h"
#include <Urho3D/IO/Log.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/GraphicsImpl.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <pvd/PxPvd.h>

Urho3DPhysX::Physics::Physics(Context * context) : Object(context),
logErrors_(true),
foundation_(nullptr),
cpuDispatcher_(nullptr),
cudaManager_(nullptr),
cooking_(nullptr),
errorCallback_(this),
defBroadPhaseType_(PxBroadPhaseType::Enum::eABP),
#ifdef _DEBUG
defEnableGPUDynamics_(false),
#else
defEnableGPUDynamics_(true),
#endif
defUseCCD_(true),
pvdTransport_(nullptr),
pvd_(nullptr)
{
}

Urho3DPhysX::Physics::~Physics()
{
    if (triangleMeshesShapes_.Size())
    {
        for (HashMap<Pair<StringHash, unsigned>, PxTriangleMesh*>::ConstIterator i = triangleMeshesShapes_.Begin(); i != triangleMeshesShapes_.End(); ++i)
        {
            i->second_->release();
        }
    }
    if (convexMeshesShapes_.Size())
    {
        for (HashMap<Pair<StringHash, unsigned>, PxConvexMesh*>::ConstIterator i = convexMeshesShapes_.Begin(); i != convexMeshesShapes_.End(); ++i)
        {
            i->second_->release();
        }
    }
    if (defaultMaterial_)
        defaultMaterial_.Reset();
    if (cooking_)
        cooking_->release();
    if (physics_)
        physics_->release();
    if (foundation_)
        foundation_->release();
    if (pvd_)
        pvd_->release();
    if (pvdTransport_)
        pvdTransport_->release();
    PxCloseExtensions();
}

bool Urho3DPhysX::Physics::InitializePhysX()
{
    foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, defaultAllocator_, errorCallback_);
    if (!foundation_)
    {
        URHO3D_LOGERROR("Failed to create PxFoundation.");
        return false;
    }
    pvd_ = PxCreatePvd(*foundation_);
    pvdTransport_ = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    pvd_->connect(*pvdTransport_, PxPvdInstrumentationFlag::eALL);
    PxTolerancesScale scale;
    physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, scale, false, pvd_);
    if (!physics_)
    {
        URHO3D_LOGERROR("Failed to create PxPhysics.");
        return false;
    }
    PxInitExtensions(*physics_, pvd_);
    cpuDispatcher_ = PxDefaultCpuDispatcherCreate(GetNumLogicalCPUs());
    PxCudaContextManagerDesc descr;
#ifdef URHO3D_OPENGL
    descr.graphicsDevice = GetSubsystem<Graphics>()->GetImpl()->GetGLContext();
#else
    descr.graphicsDevice = GetSubsystem<Graphics>()->GetImpl()->GetDevice();
#endif
    cudaManager_ = PxCreateCudaContextManager(*foundation_, descr);
    //initialize cooking
    PxCookingParams cookingParams(scale);
    cookingParams.buildGPUData = true;

    cooking_ = PxCreateCooking(PX_PHYSICS_VERSION, *foundation_, cookingParams);
    //create default material, TODO: this needs improvement
    defaultMaterial_ = SharedPtr<PhysXMaterial>(new PhysXMaterial(context_));
    defaultMaterial_->SetName("DefaultPxMaterial");
    GetSubsystem<ResourceCache>()->AddManualResource(defaultMaterial_);
    URHO3D_LOGINFO("Initialized PhysX.");
    return true;
}

void Urho3DPhysX::Physics::SetDefaultMaterial(PhysXMaterial * value)
{
    if (value && value != defaultMaterial_)
    {
        defaultMaterial_ = SharedPtr<PhysXMaterial>(value);
    }
}

PxTriangleMesh * Urho3DPhysX::Physics::GetOrCreateTriangleMesh(const String & name, unsigned lodLevel)
{
    return GetOrCreateTriangleMesh(GetSubsystem<ResourceCache>()->GetResource<Model>(name), lodLevel);
}

PxTriangleMesh * Urho3DPhysX::Physics::GetOrCreateTriangleMesh(Model * source, unsigned lodLevel)
{
    if (source)
    {
        PxTriangleMesh* ret(nullptr);
        Pair<StringHash, unsigned> key(source->GetNameHash(), lodLevel);
        if (triangleMeshesShapes_.TryGetValue(key, ret))
        {
            return ret;
        }
        else
        {
            //combined index data for indicies
            PODVector<unsigned short> combinedIndexData;
            //combined index data for large indicies
            PODVector<unsigned> combinedLargeIndexData;
            PODVector<Vector3> verticies;
            bool indexSizeSet = false;
            bool isUsingLargeIndicies = false;
            for (unsigned i = 0; i < source->GetNumGeometries(); ++i)
            {
                Geometry* geometry = source->GetGeometry(i, lodLevel);
                const unsigned char* vertexData;
                const unsigned char* indexData;
                unsigned vertexSize;
                unsigned indexSize;
                const PODVector<VertexElement>* elements;
                geometry->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
                if (!vertexData || VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION) != 0)
                {
                    __debugbreak();
                }
                unsigned vertexStart = geometry->GetVertexStart();
                unsigned vertexCount = geometry->GetVertexCount();
                for (unsigned j = 0; j < vertexCount; ++j)
                {
                    verticies.Push(*((const Vector3*)(&vertexData[(vertexStart + j) * vertexSize])));
                }
                if (!indexSizeSet)
                {
                    if (indexSize == sizeof(unsigned))
                        isUsingLargeIndicies = true;
                    indexSizeSet = true;
                }
                unsigned indexStart = geometry->GetIndexStart();
                unsigned indexCount = geometry->GetIndexCount();
                for (unsigned k = 0; k < indexCount; ++k)
                {
                    if (!isUsingLargeIndicies)
                        combinedIndexData.Push(*(const unsigned short*)(&indexData[(k + indexStart) * indexSize]));
                    else
                        combinedLargeIndexData.Push(*(const unsigned*)(&indexData[(k + indexStart) * indexSize]));
                }
            }
            PxTriangleMeshDesc descr;
            descr.points.count = verticies.Size();
            descr.points.stride = sizeof(Vector3);
            descr.points.data = verticies.Buffer();

            descr.triangles.count = isUsingLargeIndicies ? combinedLargeIndexData.Size() / 3 : combinedIndexData.Size() / 3;
            descr.triangles.stride = isUsingLargeIndicies ? sizeof(unsigned) * 3 : sizeof(unsigned short) * 3;
            descr.triangles.data = isUsingLargeIndicies ? (void*)combinedLargeIndexData.Buffer() : (void*)combinedIndexData.Buffer();
            if(!isUsingLargeIndicies)
                descr.flags |= PxMeshFlag::e16_BIT_INDICES;

            PxDefaultMemoryOutputStream writeBuffer;
            PxTriangleMeshCookingResult::Enum result;
            bool status = cooking_->cookTriangleMesh(descr, writeBuffer, &result);
            if (!status)
            {
                return nullptr;
            }
            PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
            ret = physics_->createTriangleMesh(readBuffer);
            triangleMeshesShapes_.Insert(MakePair(key, ret));
            return ret;            
        }
    }
    return nullptr;
}

PxConvexMesh * Urho3DPhysX::Physics::GetOrCreateConvexMesh(const String & name, unsigned lodLevel)
{
    return GetOrCreateConvexMesh(GetSubsystem<ResourceCache>()->GetResource<Model>(name), lodLevel);
}

PxConvexMesh * Urho3DPhysX::Physics::GetOrCreateConvexMesh(Model * source, unsigned lodLevel)
{
    if (source)
    {
        PxConvexMesh* ret(nullptr);
        Pair<StringHash, unsigned> key(source->GetNameHash(), lodLevel);
        if (convexMeshesShapes_.TryGetValue(key, ret))
        {
            return ret;
        }
        else
        {
            PODVector<Vector3> verticies;
            unsigned numGeometries = source->GetNumGeometries();
            for (unsigned i = 0; i < numGeometries; ++i)
            {
                Geometry* geometry = source->GetGeometry(i, lodLevel);
                const unsigned char* vertexData;
                const unsigned char* indexData;
                unsigned vertexSize;
                unsigned indexSize;
                const PODVector<VertexElement>* elements;
                geometry->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
                if (!vertexData || VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION) != 0)
                {
                    __debugbreak();
                }
                unsigned vertexStart = geometry->GetVertexStart();
                unsigned vertexCount = geometry->GetVertexCount();
                for (unsigned i = 0; i < vertexCount; ++i)
                {
                    verticies.Push(*((const Vector3*)(&vertexData[(vertexStart + i) * vertexSize])));
                }
            }
            PxConvexMeshDesc descr;
            descr.points.count = verticies.Size();
            descr.points.stride = sizeof(Vector3);
            descr.points.data = verticies.Buffer();
            descr.flags |= PxConvexFlag::eCOMPUTE_CONVEX;
            descr.flags |= PxConvexFlag::eGPU_COMPATIBLE;
            PxDefaultMemoryOutputStream writeBuffer;
            if (!cooking_->cookConvexMesh(descr, writeBuffer))
            {
                return nullptr;
            }
            PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
            ret = physics_->createConvexMesh(readBuffer);
            convexMeshesShapes_.Insert(MakePair(key, ret));
            return ret;
        }
    }
    return nullptr;
}

void Urho3DPhysX::Physics::SendErrorEvent(int code, const String & message, const String& file, int line)
{
    if (IsLoggingErrors())
        URHO3D_LOGERROR("PhysX error, code: " + String(code) + " " + message);
    using namespace PhysXError;
    VariantMap& eventData = GetEventDataMap();
    eventData[P_CODE] = code;
    eventData[P_MESSAGE] = message;
    eventData[P_FILE] = file;
    eventData[P_LINE] = line;
    SendEvent(E_PHYSXERROR, eventData);
}

void Urho3DPhysX::RegisterPhysXLibrary(Context* context)
{
    PhysXScene::RegisterObject(context);
    RigidActor::RegisterObject(context);
    RigidBody::RegisterObject(context);
    StaticBody::RegisterObject(context);
    DynamicBody::RegisterObject(context);
    PhysXMaterial::RegisterObject(context);
    CollisionShape::RegisterObject(context);
    GroundPlane::RegisterObject(context);
    Joint::RegisterObject(context);
    SphericalJoint::RegisterObject(context);
    DistanceJoint::RegisterObject(context);
    FixedJoint::RegisterObject(context);
    PrismaticJoint::RegisterObject(context);
    RevoluteJoint::RegisterObject(context);
    KinematicController::RegisterObject(context);
}
