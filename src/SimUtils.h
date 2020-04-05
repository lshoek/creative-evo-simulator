#pragma once
#include "bullet/btBulletDynamicsCommon.h"
#include "ofVectorMath.h"

static class SimUtils
{
public:

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

        //glm::mat4 m = glm::mat4(0);
        //const btMatrix3x3& basis = t.getBasis();

        //// rotation
        //for (int r = 0; r < 3; r++)
        //{
        //    for (int c = 0; c < 3; c++)
        //    {
        //        m[c][r] = basis[r][c];
        //    }
        //}

        //// translation
        //btVector3 origin = t.getOrigin();
        //m[3][0] = origin.getX();
        //m[3][1] = origin.getY();
        //m[3][2] = origin.getZ();

        //// unit scale
        //m[0][3] = 0.0f;
        //m[1][3] = 0.0f;
        //m[2][3] = 0.0f;
        //m[3][3] = 1.0f;

        return m;
    }
};
