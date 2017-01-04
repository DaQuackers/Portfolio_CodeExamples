#pragma once

#include <math.h>
#include "../CapsuleSphereLab/vec3.h"
#include "../CapsuleSphereLab/matrix4.h"

struct Plane
{
	vec3f normal;
	float offset;
};

struct AABB
{
	vec3f min;
	vec3f max;
};

struct Frustum
{	
	Plane planes[6];
	vec3f corners[8];
};

struct Segment
{
	vec3f m_Start;
	vec3f m_End;
};

struct Sphere
{
	vec3f m_Center;
	float m_Radius;
};

struct Capsule
{
	Segment m_Segment;
	float m_Radius;
};

enum FrustumCorners{ FTL = 0, FBL, FBR, FTR, NTL, NTR, NBR, NBL };
enum FrustumPlanes{ NEAR_PLANE = 0, FAR_PLANE, LEFT_PLANE, RIGHT_PLANE, TOP_PLANE, BOTTOM_PLANE };

void ComputePlane(Plane &plane, const vec3f& pointA, const vec3f& pointB, const vec3f &pointC);

int ClassifyPointToPlane(const Plane& plane, const vec3f& point);

int ClassifySphereToPlane(const Plane& plane, const Sphere& sphere);

int ClassifyAabbToPlane(const Plane& plane, const AABB& aabb);

int ClassifyCapsuleToPlane(const Plane& plane, const Capsule& capsule);

void BuildFrustum( Frustum& frustum, float fov, float nearDist, float farDist, float ratio, const matrix4f& camXform );

bool FrustumToSphere(const Frustum& frustum, const Sphere& sphere);

bool FrustumToAABB(const Frustum& frustum, const AABB& aabb);

bool FrustumToCapsule(const Frustum& frustum, const Capsule& capsule);

bool AABBtoAABB(const AABB& lhs, const AABB& rhs);

bool SphereToSphere(const Sphere& lhs, const Sphere& rhs);

bool SphereToAABB(const Sphere& lhs, const AABB& rhs);

bool CapsuleToSphere(const Capsule& capsule, const Sphere& sphere);

struct Triangle
{
	vec3f V[3];
	vec3f N;
};

// SphereToTriangle
// Used for Test #1, enabled through hitting the '1' key
//
// This function takes in a Sphere and a Triangle and should
// simply determine, true or false, if there is a collision.
// 
// Return the boolean result.
//
// Notes:
// As an early out, verify that the sphere is touching the plane
// of the triangle. If it does not touch the plane, there can be
// no collision with the triangle on that plane.
bool SphereToTriangle(const Sphere& sphere, const Triangle& tri);

// SphereToTriangle
// Used for Test #2, enabled through hitting the '2' key
//
// This function takes in a Sphere and a Triangle and should
// determine, true or false, if there is a collision.
// If there is a collision, the proper displacement/correction vector
// should be calculated. This is the vector to translate the sphere
// along to push it out of the triangle. Store the result in "displacement".
//
// Return the boolean result.
//
// Notes:
// The code for this test is almost exactly the same as the code for the first test.
// Differences will be:
//	- Early out
//		+ If the sphere doesn't touch the plane OR its center is behind the plane,
//			there can be no collision.
//	- Displacement calculation
//		+ If there is a collision, calculate the displacement vector.
//			This is the contact normal scaled by the penetration depth.
//			See slides on collision reaction for the Sphere to Triangle collision test.
bool SphereToTriangle(const Sphere& sphere, const Triangle& tri, vec3f& displacement);
