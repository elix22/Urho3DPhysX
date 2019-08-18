#pragma once

#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Quaternion.h>
#include <Urho3D/Math/Matrix3x4.h>

#include <foundation/PxVec2.h>
#include <foundation/PxVec3.h>
#include <foundation/PxVec4.h>
#include <foundation/PxQuat.h>
#include <foundation/PxTransform.h>

using namespace Urho3D;
using namespace physx;

namespace Urho3DPhysX
{
    inline PxVec3 ToPxVec3(const Vector3& vector)
    {
        return PxVec3(vector.x_, vector.y_, vector.z_);
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

    inline Vector3 ToVector3(PxVec3 vector)
    {
        return Vector3(vector.x, vector.y, vector.z);
    }

    inline Quaternion ToQuaternion(PxQuat quat)
    {
        return Quaternion(quat.w, quat.x, quat.y, quat.z);
    }
}
