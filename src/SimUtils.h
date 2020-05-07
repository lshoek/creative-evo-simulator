#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "bullet/LinearMath/btTransform.h"
#include "bullet/LinearMath/btVector3.h"
#include "ofVectorMath.h"

#define SIMD_PI_2 ((SIMD_PI)*0.5)
#define SIMD_PI_4 ((SIMD_PI)*0.25)
#define SIMD_PI_8 ((SIMD_PI)*0.125)

static class SimUtils
{
public:
	class b3RefFrameHelper
	{
	public:
		static btVector3 getPointWorldToLocal(const btTransform& localObjectCenterOfMassTransform, const btVector3& point) {
			return localObjectCenterOfMassTransform.inverse() * point;
		}

		static btVector3 getPointLocalToWorld(const btTransform& localObjectCenterOfMassTransform, const btVector3& point) {
			return localObjectCenterOfMassTransform * point;
		}

		static btVector3 getAxisWorldToLocal(const btTransform& localObjectCenterOfMassTransform, const btVector3& axis) {
			btTransform local1 = localObjectCenterOfMassTransform.inverse();
			btVector3 zero(0, 0, 0);
			local1.setOrigin(zero);
			return local1 * axis;
		}

		static btVector3 getAxisLocalToWorld(const btTransform& localObjectCenterOfMassTransform, const btVector3& axis) {
			btTransform local1 = localObjectCenterOfMassTransform;
			btVector3 zero(0, 0, 0);
			local1.setOrigin(zero);
			return local1 * axis;
		}

		static btTransform getTransformWorldToLocal(const btTransform& localObjectCenterOfMassTransform, const btTransform& transform) {
			return localObjectCenterOfMassTransform.inverse() * transform;
		}

		static btTransform getTransformLocalToWorld(const btTransform& localObjectCenterOfMassTransform, const btTransform& transform) {
			return localObjectCenterOfMassTransform * transform;
		}

	};

	static btVector3 sign(btVector3 v)
	{
		return btVector3(v.x() >= 0 ? 1. : -1., v.y() >= 0 ? 1. : -1., v.z() >= 0 ? 1. : -1.);
	}

	static btVector3 flip(btVector3 v)
	{
		return btVector3(v.x() > 0 ? 0. : 1., v.y() > 0 ? 0. : 1., v.z() > 0 ? 0. : 1.);
	}

	// Origin lies inside the box so there is always an intersection
	static btScalar distToSurface(btVector3 dir, btVector3 boxExtents, btVector3& perp)
	{
		btVector3 o = btVector3(0, 0, 0);
		btVector3 surf = btVector3(1, 0, 0);

		btScalar tmin = (-boxExtents.x() - o.x()) / dir.x();
		btScalar tmax = (boxExtents.x() - o.x()) / dir.x();
		if (tmin > tmax) std::swap(tmin, tmax);

		btScalar tymin = (-boxExtents.y() - o.y()) / dir.y();
		btScalar tymax = (boxExtents.y() - o.y()) / dir.y();
		if (tymin > tymax) std::swap(tymin, tymax);

		if (tymin > tmin) {
			tmin = tymin;
			surf = btVector3(0, 1, 0);
		}
		if (tymax < tmax) tmax = tymax;

		btScalar tzmin = (-boxExtents.z() - o.z()) / dir.z();
		btScalar tzmax = (boxExtents.z() - o.z()) / dir.z();
		if (tzmin > tzmax) std::swap(tzmin, tzmax);

		if (tzmin > tmin) {
			tmin = tzmin;
			surf = btVector3(0, 0, 1);
		}
		if (tzmax < tmax) tmax = tzmax;

		btScalar t = tmin;
		
		// perp is perpendicular to the surface 
		perp = surf * sign(dir);

		// tmin should always be positive as the intersection is in front of the origin of the ray
		return t;
	}

	// Conversion functions
    static glm::vec3 bulletToGlm(const btVector3& v) { return glm::vec3(v.getX(), v.getY(), v.getZ()); }
    static btVector3 glmToBullet(const glm::vec3& v) { return btVector3(v.x, v.y, v.z); }
    static glm::quat bulletToGlm(const btQuaternion& q) { return glm::quat(q.getW(), q.getX(), q.getY(), q.getZ()); }
    static btQuaternion glmToBullet(const glm::quat& q) { return btQuaternion(q.x, q.y, q.z, q.w); }
    static btMatrix3x3 glmToBullet(const glm::mat3& m) { return btMatrix3x3(m[0][0], m[1][0], m[2][0], m[0][1], m[1][1], m[2][1], m[0][2], m[1][2], m[2][2]); }

    // btTransform does not contain a full 4x4 matrix, so this transform is lossy.
    // Affine transformations are OK but perspective transformations are not.
    static btTransform glmToBullet(const glm::mat4& m)
    {
        glm::mat3 m3(m);
        return btTransform(glmToBullet(m3), glmToBullet(glm::vec3(m[3][0], m[3][1], m[3][2])));
    }

    static glm::mat4 bulletToGlm(const btTransform& t)
    {
        glm::mat4 m;
        t.getOpenGLMatrix(glm::value_ptr(m));
        return m;
    }
};
