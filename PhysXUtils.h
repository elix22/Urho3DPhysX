#pragma once

#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Quaternion.h>
#include <Urho3D/Math/Matrix3x4.h>

#include <foundation/PxVec2.h>
#include <foundation/PxVec3.h>
#include <foundation/PxVec4.h>
#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>
#include <characterkinematic/PxExtended.h>

using namespace Urho3D;
using namespace physx;

#ifndef URHOPX_API
    #ifdef Urho3DPhysX_EXPORTS
        #define URHOPX_API __declspec(dllexport)
    #else
        #define URHOPX_API __declspec(dllimport)
#endif // Urho3DPhysX_EXPORTS

#endif
namespace Urho3DPhysX
{
    inline PxVec3 ToPxVec3(const Vector3& vector)
    {
        return PxVec3(vector.x_, vector.y_, vector.z_);
    }

    inline PxExtendedVec3 ToPxExtendedVec3(const Vector3& vector)
    {
        return PxExtendedVec3((PxExtended)vector.x_, (PxExtended)vector.y_, (PxExtended)vector.z_);
    }

    inline PxQuat ToPxQuat(const Quaternion& quat)
    {
        return PxQuat(quat.x_, quat.y_, quat.z_, quat.w_);
    }

    inline PxTransform ToPxTransform(const Vector3& vector, const Quaternion& quaternion = Quaternion::IDENTITY)
    {
        return PxTransform(ToPxVec3(vector), ToPxQuat(quaternion));
    }

    inline PxTransform ToPxTransform(const Quaternion& quaternion)
    {
        return PxTransform(ToPxQuat(quaternion));
    }

    inline PxTransform ToPxTransform(const Matrix3x4& matrix)
    {
        return ToPxTransform(matrix.Translation(), matrix.Rotation());
    }

    inline Vector3 ToVector3(const PxVec3& vector)
    {
        return Vector3(vector.x, vector.y, vector.z);
    }

    inline Vector3 ToVector3(const PxExtendedVec3& vector)
    {
        return Vector3((float)vector.x, (float)vector.y, (float)vector.z);
    }

    inline Quaternion ToQuaternion(const PxQuat& quat)
    {
        return Quaternion(quat.w, quat.x, quat.y, quat.z);
    }
}
