#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "bullet/LinearMath/btTransform.h"
#include "bullet/LinearMath/btVector3.h"
#include "ofVectorMath.h"

#define AnonymousTag 0
#define BodyTag 1
#define JointTag 2
#define BrushTag 3
#define TerrainTag 4
#define CanvasTag 5

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
			return localObjectCenterOfMassTransform.inverse() * point; // transforms the point from the world frame into the local frame
		}

		static btVector3 getPointLocalToWorld(const btTransform& localObjectCenterOfMassTransform, const btVector3& point) {
			return localObjectCenterOfMassTransform * point; // transforms the point from the world frame into the local frame
		}

		static btVector3 getAxisWorldToLocal(const btTransform& localObjectCenterOfMassTransform, const btVector3& axis) {
			btTransform local1 = localObjectCenterOfMassTransform.inverse(); // transforms the axis from the local frame into the world frame
			btVector3 zero(0, 0, 0);
			local1.setOrigin(zero);
			return local1 * axis;
		}

		static btVector3 getAxisLocalToWorld(const btTransform& localObjectCenterOfMassTransform, const btVector3& axis) {
			btTransform local1 = localObjectCenterOfMassTransform; // transforms the axis from the local frame into the world frame
			btVector3 zero(0, 0, 0);
			local1.setOrigin(zero);
			return local1 * axis;
		}

		static btTransform getTransformWorldToLocal(const btTransform& localObjectCenterOfMassTransform, const btTransform& transform) {
			return localObjectCenterOfMassTransform.inverse() * transform; // transforms the axis from the local frame into the world frame
		}

		static btTransform getTransformLocalToWorld(const btTransform& localObjectCenterOfMassTransform, const btTransform& transform) {
			return localObjectCenterOfMassTransform * transform; // transforms the axis from the local frame into the world frame
		}

	};

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
