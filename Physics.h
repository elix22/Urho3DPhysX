#pragma once
#include "PhysXEvents.h"
#include <Urho3D/Core/Object.h>
#include <PxPhysicsAPI.h>

namespace Urho3D
{
    class Model;
}
using namespace Urho3D;
using namespace physx;

namespace Urho3DPhysX
{
    class PhysXMaterial;

    class __declspec(dllexport) Physics : public Object
    {
        URHO3D_OBJECT(Physics, Object);

    public:
        Physics(Context* context);
        ~Physics();

        ///Initialize Physx sdk
        bool InitializePhysX();
        ///
        PxFoundation* GetFoundation() { return foundation_; }
        ///
        PxPhysics* GetPhysics() { return physics_; }
        ///
        PxCpuDispatcher* GetCpuDispatcher() { return cpuDispatcher_; }
        ///
        PxCudaContextManager* GetCUDAContextManager() { return cudaManager_; }
        ///
        PhysXMaterial* GetDefaultMaterial() { return defaultMaterial_; }
        ///
        void SetDefaultMaterial(PhysXMaterial* value);
        ///Get Triangle mesh or create new one if not created yet
        PxTriangleMesh* GetOrCreateTriangleMesh(const String& name, unsigned lodLevel);
        PxTriangleMesh* GetOrCreateTriangleMesh(Model* source, unsigned lodLevel);
        ///Get convex mesh or create new one if not created yet
        PxConvexMesh* GetOrCreateConvexMesh(const String& name, unsigned lodLevel);
        ///Get convex mesh or create new one if not created yet
        PxConvexMesh* GetOrCreateConvexMesh(Model* source, unsigned lodLevel);
        ///
        const ErrorCallback& GetErrorCallback() const { return errorCallback_; }
        ///Send error event, called from error callback
        void SendErrorEvent(int code, const String& message, const String& file, int line);
        ///
        bool IsLoggingErrors() const { return logErrors_; }
        ///
        void SetLogErrors(bool val) { logErrors_ = val; }
        ///
        void SetDefBraodPhaseType(PxBroadPhaseType::Enum type) { defBroadPhaseType_ = type; }
        ///
        PxBroadPhaseType::Enum GetDefBroadPhaseType() const {return defBroadPhaseType_; }
        ///
        void SetDefEnableGPUDynamics(bool val) { defEnableGPUDynamics_ = val; }
        ///
        bool GetDefEnableGPUDynamics() const { return defEnableGPUDynamics_; }
        ///
        void SetDefUseCCD(bool val) { defUseCCD_ = val; }
        ///
        bool GetDefUseCCD() const { return defUseCCD_; }

    private:
        PxFoundation* foundation_;
        PxPhysics* physics_;
        PxDefaultAllocator defaultAllocator_;
        PxCpuDispatcher* cpuDispatcher_;
        PxCudaContextManager* cudaManager_;
        PxCooking* cooking_;
        ///default material
        SharedPtr<PhysXMaterial> defaultMaterial_;
        ///triangle meshes
        HashMap<Pair<StringHash, unsigned>, PxTriangleMesh*> triangleMeshesShapes_;
        ///convex meshes
        HashMap<Pair<StringHash, unsigned>, PxConvexMesh*> convexMeshesShapes_;
        ///callbacks
        ErrorCallback errorCallback_;
        ///If enabled, error messagess will be logged immidietly. True by defualt
        bool logErrors_;
        ///Defaults
        PxBroadPhaseType::Enum defBroadPhaseType_;
        bool defEnableGPUDynamics_;
        bool defUseCCD_;

        ///visual debugger
        //PxPvd* pvd_;
    };
    ///Register Physx objects
    void __declspec(dllexport) RegisterPhysXLibrary(Context* context);
}