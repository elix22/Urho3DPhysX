# Urho3DPhysX
## [Nvidia PhysX](https://github.com/NVIDIAGameWorks/PhysX) addon for [Urho3D](https://urho3d.github.io/) engine.<br/>
*Work in progess- WARNING: This addon is under development and may not be production ready, at the current state basic functionality is provided, but there may be some serious issues.* <br/>

### Setup
Download and [build Urho3D engine](https://urho3d.github.io/documentation/HEAD/_building.html) with bullet physics disabled (URHO3D_PHYSICS = 0).

Build PhysX 4.1 SDK following these steps (*instructions only for Windows, 64bit and Visual Studio 2017, users of different operating systems will have to adjust these steps according to the system they're using*):
1. Download PhysX 4.1 SDK.
2. Go to  "(Physx SDK root)\physx\buildtools\presets\public\vc15win64.xml" and set "NV_USE_STATIC_WINCRT" to "False".
3. In the same file add: \<cmakeSwitch name="NV_APPEND_CONFIG_NAME" value="True" comment="Append config " />
4. Follow instructions from [here](https://github.com/NVIDIAGameWorks/PhysX) to build PhysX.

To build whole project (addon as library, samples application and modified Urho3DPlayer application) copy 'bin/Data' and 'bin/CoreData' folders from Urho3D directory into 'bin' directory and proceed as ususal when [using Urho3D library](https://urho3d.github.io/documentation/HEAD/_using_library.html). *(Or just copy source files to Your project.)*

Currently there's no script for automatic linking, so PhysX must be linked manually:

1. Add include directories: "(PhysX SDK root)\physx\include" and "(PhysX SDK root)\pxshared\include";
2. Link libraries: PhysX; PhysXCommon; PhysXCooking; PhysXFoundation; PhysXExtensions and copy proper .dll files (for both release and debug) to the executable directory;
3. To be able to use GPU acceleration copy also "PhysXGpu_64.dll" and "PhysXDevice64.dll"

### Using Addon
1. Create 'Physics' subsystem (context_->RegisterSubsystem\<Physics\>());
2. call Physics::InitializePhysX()
3. Optionally set default values for scene creation and PhysXMaterial (see below for more info).

*See: [Sample application](https://github.com/lezak/Urho3DPhysX/blob/0ec905019ba917e6371e340c81000b1dbabcc03d/Samples/UrhoPhysXSamples.cpp#L38) initialization.* 

Urho's "PhysicsWorld" component is replaced by "PhysXScene" component.
There are some customization options availble by setting following scene vars **before** PhysXScene component is created (it's not possible to change it later):
1. [broadphase type](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/RigidBodyCollision.html#broad-phase-algorithms): Var name: "BPT", value: String; scene->SetVar("BPT", "SAP"/"ABP"/"GPU") where "SAP" - sweep-and-prune; "ABP"- automatic box pruning; "GPU" - GPU; *multi box pruning* is currently not supported (will be added soon)
2. GPU dynamics: Var name: "GPUD", value: bool; scene->SetVar("GPUD", true/false)
3. Continuous collision detection: Var name: "CCD", value: bool; scene->SetVar("CCD", true/false)

Alternatively default values for these paramters can be set in Physics subsystem (Physics::SetDefBraodPhaseType; Physics::SetDefEnableGPUDynamics; Physics::SetDefUseCCD). Default values will be used when a new PhysXScene component is created in a scene without vars set up, each PhysXScene can be set up differently.

When physics object (for example dynamic body) is created in a scene without PhysXWorld, component will be created automatically, so make sure that proper options are already set. 


Addon recreates PhysX inheritance hierarchy, so there are some components that should NOT be used (they're available because Urho doesn't allow to register pure virtual classes): 
1. Create <u>DynamicBody and StaticBody</u>, don't create RigidActor and <u>RigidBody</u>
2. Don't create Joint component, use specific derived components 

**Available Collision Shapes:**
- Box;
- Sphere;
- Plane;
- Capsule;
- Convex Mesh;
- Triangle Mesh;

*Height field (terrain) is not supported at the moment* 

*Currently collision shapes are unique and cannot be shared between objects, this will propably change in the future.*

**Triggers and collision filtering**

Unlike Urho's default physics, triggers and collision layer/mask are set on CollisionShape and not on physics object.

**PhysX materials**

PhysX Materials are implemnted as a resources applied to a CollisionShape. Material defines dynamic friction, static friction and restitution. Definition looks like this:
```
<PxMaterial>
    <dynamicFriction value="float" />
    <staticFriction value="float" />
    <restitution value="float" />
</PxMaterial>
```
Collision shape without material set will use default material that can be set in the Physics subsystem (Physcs::SetDefaultMaterial).


**Available joints:**

- Distance
- Fixed
- Prismatic (slider)
- Revolute
- Spherical 

**Events**

[List of available events and paramters](https://github.com/lezak/Urho3DPhysX/blob/0ec905019ba917e6371e340c81000b1dbabcc03d/PhysXEvents.h#L9) 

**Scene queries**

- Raycast/RaycastSingle
- SphereCast/SphereCastSingle
- BoxCast/BoxCastSingle
- CapsuleCast/CapsuleCastSingle


*At the moment queries for more then first hit return fixed number of results that can be adjusted [here](https://github.com/lezak/Urho3DPhysX/blob/0ec905019ba917e6371e340c81000b1dbabcc03d/PhysXScene.cpp#L19) as needed.*






