#include "CollisionLibrary.h"
#include <math.h>

// ComputePlane
//
// Calculate the plane normal and plane offset from the input points
void ComputePlane(Plane &plane, const vec3f& pointA, const vec3f& pointB, const vec3f &pointC)
{
	cross_product(plane.normal, pointB - pointA, pointC - pointB);
	plane.normal.normalize();
	plane.offset = dot_product(plane.normal, pointA);
}

// ClassifyPointToPlane
//
// Perform a half-space test. Returns 1 if the point is on or in front of the plane.
// Returns 2 if the point is behind the plane.
int ClassifyPointToPlane(const Plane& plane, const vec3f& point)
{
	if (dot_product(plane.normal, point) < plane.offset)
		return 2;
	return 1;
}

// ClassifySphereToPlane
//
// Perform a sphere-to-plane test. 
// Returns 1 if the sphere is in front of the plane.
// Returns 2 if the sphere is behind the plane.
// Returns 3 if the sphere straddles the plane.
int ClassifySphereToPlane(const Plane& plane, const Sphere& sphere)
{
	if ((dot_product(sphere.m_Center, plane.normal) - plane.offset) > sphere.m_Radius)
		return 1;
	else if ((dot_product(sphere.m_Center, plane.normal) - plane.offset) < -sphere.m_Radius)
		return 2;
	return 3;
}

// ClassifyAabbToPlane
//
// Performs a AABB-to-plane test.
// Returns 1 if the aabb is in front of the plane.
// Returns 2 if the aabb is behind the plane.
// Returns 3 if the aabb straddles the plane.
int ClassifyAabbToPlane(const Plane& plane, const AABB& aabb)
{
	vec3f CP = (aabb.min + aabb.max)*0.5;
	vec3f Ext = (aabb.max - CP);
	vec3f nrm = vec3f(abs(plane.normal.x), abs(plane.normal.y), abs(plane.normal.z));
	float radius = dot_product(Ext, nrm);
	if ((dot_product(CP, plane.normal) - plane.offset) > radius)
		return 1;
	else if ((dot_product(CP, plane.normal) - plane.offset) < -radius)
		return 2;
	return 3;
}

// ClassifyCapsuleToPlane
//
// Performs a Capsule-to-plane test.
// Returns 1 if the aabb is in front of the plane.
// Returns 2 if the aabb is behind the plane.
// Returns 3 if the aabb straddles the plane.
int ClassifyCapsuleToPlane(const Plane& plane, const Capsule& capsule)
{
	int Spt1, Spt2;
	Sphere sphere1; sphere1.m_Center = capsule.m_Segment.m_Start;
	Sphere sphere2; sphere2.m_Center = capsule.m_Segment.m_End;
	sphere1.m_Radius = sphere2.m_Radius = capsule.m_Radius;
	Spt1 = ClassifySphereToPlane(plane, sphere1);
	Spt2 = ClassifySphereToPlane(plane, sphere2);
	if (Spt1 == 1 && Spt2 == 1)
		return Spt1;
	if (Spt1 == 2 && Spt2 == 2)
		return Spt2;
	return 3;
}

// BuildFrustum
//
// Calculates the corner points and planes of the frustum based upon input values.
void BuildFrustum(Frustum& frustum, float fov, float nearDist, float farDist, float ratio, const matrix4f& camXform)
{
	// TO DO:
	// Calculate the 8 corner points of the frustum and store them in the frustum.corners[] array.
	// Use the FrustumCorners enum in CollisionLibrary.h to index into the corners array.
	vec3f NcP, FcP;
	NcP = camXform.axis_pos - camXform.axis_z * nearDist;
	FcP = camXform.axis_pos - camXform.axis_z * farDist;
	float Hnear = 2 * tanf(fov*0.5)*nearDist;
	float Hfar = 2 * tanf(fov*0.5)*farDist;
	float Wnear = Hnear*ratio;
	float Wfar = Hfar*ratio;
	frustum.corners[FTL] = FcP + camXform.axis_y*(Hfar*0.5) - camXform.axis_x*(Wfar*0.5);
	frustum.corners[FTR] = FcP + camXform.axis_y*(Hfar*0.5) + camXform.axis_x*(Wfar*0.5);
	frustum.corners[FBL] = FcP - camXform.axis_y*(Hfar*0.5) - camXform.axis_x*(Wfar*0.5);
	frustum.corners[FBR] = FcP - camXform.axis_y*(Hfar*0.5) + camXform.axis_x*(Wfar*0.5);
	frustum.corners[NTL] = NcP + camXform.axis_y*(Hnear*0.5) - camXform.axis_x*(Wnear*0.5);
	frustum.corners[NTR] = NcP + camXform.axis_y*(Hnear*0.5) + camXform.axis_x*(Wnear*0.5);
	frustum.corners[NBL] = NcP - camXform.axis_y*(Hnear*0.5) - camXform.axis_x*(Wnear*0.5);
	frustum.corners[NBR] = NcP - camXform.axis_y*(Hnear*0.5) + camXform.axis_x*(Wnear*0.5);

	// Use the corner points to calculate the frustum planes.
	// This step is completed for you.
	ComputePlane(frustum.planes[NEAR_PLANE], frustum.corners[NBR], frustum.corners[NBL], frustum.corners[NTL]);
	ComputePlane(frustum.planes[FAR_PLANE], frustum.corners[FBL], frustum.corners[FBR], frustum.corners[FTR]);
	ComputePlane(frustum.planes[LEFT_PLANE], frustum.corners[NBL], frustum.corners[FBL], frustum.corners[FTL]);
	ComputePlane(frustum.planes[RIGHT_PLANE], frustum.corners[FBR], frustum.corners[NBR], frustum.corners[NTR]);
	ComputePlane(frustum.planes[TOP_PLANE], frustum.corners[NTR], frustum.corners[NTL], frustum.corners[FTL]);
	ComputePlane(frustum.planes[BOTTOM_PLANE], frustum.corners[NBL], frustum.corners[NBR], frustum.corners[FBR]);
}

// FrustumToSphere
//
// Perform a Sphere-to-Frustum check. Returns true if the sphere is inside. False if not.
bool FrustumToSphere(const Frustum& frustum, const Sphere& sphere)
{
	for (int i = 5; i >= 0; --i)
	{
		if (ClassifySphereToPlane(frustum.planes[i], sphere) == 2)
			return false;
	}
	return true;
}

// FrustumToAABB
//
// Perform a Aabb-to-Frustum check. Returns true if the aabb is inside. False if not.
bool FrustumToAABB(const Frustum& frustum, const AABB& aabb)
{
	for (int i = 5; i >= 0; --i)
	{
		if (ClassifyAabbToPlane(frustum.planes[i], aabb) == 2)
			return false;
	}
	return true;
}

// FrustumToCapsule
//
// Perform a Capsule-to-Frustum check. Returns true if the Capsule is inside. False if not.
bool FrustumToCapsule(const Frustum& frustum, const Capsule& capsule)
{
	for (int i = 5; i >= 0; --i)
	{
		if (ClassifyCapsuleToPlane(frustum.planes[i], capsule) == 2)
			return false;
	}
	return true;
}

// AABBtoAABB
//
// Returns true if the AABBs collide. False if not.
bool AABBtoAABB(const AABB& lhs, const AABB& rhs)
{
	if (lhs.max.x < rhs.min.x || lhs.min.x > rhs.max.x) return false;
	if (lhs.max.y < rhs.min.y || lhs.min.y > rhs.max.y) return false;
	if (lhs.max.z < rhs.min.z || lhs.min.z > rhs.max.z) return false;
	return true;
}

// SphereToSphere
//
// Returns true if the Spheres collide. False if not.
bool SphereToSphere(const Sphere& lhs, const Sphere& rhs)
{
	vec3f df = lhs.m_Center - rhs.m_Center;
	float distance = pow(df.x, 2) + pow(df.y, 2) + pow(df.z, 2);
	if (distance < pow((lhs.m_Radius + rhs.m_Radius), 2))
		return true;
	return false;
}

// SphereToAABB
//
// Returns true if the sphere collides with the AABB. False if not.
bool SphereToAABB(const Sphere& lhs, const AABB& rhs)
{
	vec3f cpos;
	if (lhs.m_Center.x < rhs.min.x) cpos.x = rhs.min.x;
	else if (lhs.m_Center.x > rhs.max.x) cpos.x = rhs.min.x;
	else cpos.x = lhs.m_Center.x;
	if (lhs.m_Center.y < rhs.min.y) cpos.y = rhs.min.y;
	else if (lhs.m_Center.y > rhs.max.y) cpos.y = rhs.min.y;
	else cpos.y = lhs.m_Center.y;
	if (lhs.m_Center.z < rhs.min.z) cpos.z = rhs.min.z;
	else if (lhs.m_Center.z > rhs.max.z) cpos.z = rhs.min.z;
	else cpos.z = lhs.m_Center.z;
	vec3f df = lhs.m_Center - cpos;
	float dist = pow(df.x, 2) + pow(df.y, 2) + pow(df.z, 2);
	if (dist < pow(lhs.m_Radius, 2)) return true;
	return false;
}

// CapsuleToSphere
//
// Returns true if the capsule collides with the sphere. False if not.
bool CapsuleToSphere(const Capsule& capsule, const Sphere& sphere)
{
	Sphere Sp1, Sp2;  Sp1.m_Radius = Sp2.m_Radius = capsule.m_Radius;
	Sp1.m_Center = capsule.m_Segment.m_Start;
	Sp2.m_Center = capsule.m_Segment.m_End;
	if (SphereToSphere(sphere, Sp1)) return true;
	if (SphereToSphere(sphere, Sp2)) return true;

	vec3f VBs = capsule.m_Segment.m_End - capsule.m_Segment.m_Start;
	float fRatio = dot_product((sphere.m_Center - capsule.m_Segment.m_Start), VBs) / dot_product(VBs, VBs);
	if (fRatio < 0) fRatio = 0;
	if (fRatio > 1) fRatio = 1;

	vec3f CsPl = capsule.m_Segment.m_Start + (VBs*fRatio);
	vec3f df = CsPl - sphere.m_Center;
	float dist = pow(df.x, 2) + pow(df.y, 2) + pow(df.z, 2);
	if (dist < pow((sphere.m_Radius + capsule.m_Radius), 2))
		return true;
	return false;
}

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
bool SphereToTriangle(const Sphere& sphere, const Triangle& tri)
{
	float sphere_dist = dot_product(sphere.m_Center, tri.N) - dot_product(tri.N, tri.V[0]);
	if (sphere_dist > sphere.m_Radius)
		return false;
	else if (sphere_dist < -sphere.m_Radius)
		return false;
	vec3f ProjPoint = sphere.m_Center - tri.N * sphere_dist;
	vec3f ED[3], NM[3], CP[3];
	for (size_t i = 0; i < 3; i++)
	{
		// Calculate Edge and Edge Normal
		ED[i] = tri.V[(i + 1) % 3] - tri.V[i];
		cross_product(NM[i], ED[i], tri.N);
		// Calculate Closest Point on the Edge
		float D = dot_product((ProjPoint - tri.V[i]), ED[i]) / dot_product(ED[i], ED[i]);
		if (D < 0) D = 0; if (D > 1) D = 1;
		CP[i] = tri.V[i] + ((ED[i])*D);
	}
	for (size_t j = 0; j < 3; j++)
	{
		if (dot_product(NM[j], (ProjPoint - tri.V[j])) > 0)
		{
			for (size_t i = 0; i < 3; i++)
			{
				//For each edge check if its closest point is in the triangle
				vec3f vb = sphere.m_Center - CP[i];
				if (dot_product(vb,vb) < pow(sphere.m_Radius, 2))
					return true;
			}
			//otherwise their is no collision
			return false;
		}
	}
	//if you've made it this far the ProjPoint is inside the Triangle
	vec3f vb = sphere.m_Center - ProjPoint;
	if (dot_product(vb,vb) < pow(sphere.m_Radius, 2))
		return true;
}

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
bool SphereToTriangle(const Sphere& sphere, const Triangle& tri, vec3f& displacement)
{
	float sphere_dist = dot_product(sphere.m_Center, tri.N) - dot_product(tri.N, tri.V[0]);
	if (sphere_dist > sphere.m_Radius)
		return false;
	if (sphere_dist < 0)
		return false;
	vec3f ProjPoint = sphere.m_Center - tri.N * sphere_dist;
	vec3f ED[3], NM[3], CP[3];
	for (size_t i = 0; i < 3; i++)
	{
		// Calculate Edge and Edge Normal
		ED[i] = tri.V[(i + 1) % 3] - tri.V[i];
		cross_product(NM[i], ED[i], tri.N);
		// Calculate Closest Point on the Edge
		float D = dot_product((ProjPoint - tri.V[i]), ED[i]) / dot_product(ED[i], ED[i]);
		if (D < 0) D = 0; if (D > 1) D = 1;
		CP[i] = tri.V[i] + ((ED[i])*D);
	}
	for (size_t j = 0; j < 3; j++)
	{
		if (dot_product(NM[j], (ProjPoint - tri.V[j])) > 0)
		{
			float dist = FLT_MAX; size_t index;
			for (size_t i = 0; i < 3; i++)
			{
				vec3f vb = sphere.m_Center - CP[i];
				if (dot_product(vb,vb) < dist){
					dist = dot_product(vb, vb);
					index = i;
				}
			}
			ProjPoint = CP[index];
		}
	}
	//if you've made it this far the ProjPoint is inside the Triangle
	vec3f vb = sphere.m_Center - ProjPoint;
	float dist = dot_product(vb,vb);
	if (dist < pow(sphere.m_Radius, 2)){
		dist = sqrt(dist);
		if (abs(dist) > FLT_EPSILON){
			vb = vb.normalize();
			displacement = vb*(sphere.m_Radius - dist);
		}
		else
		{
			displacement = tri.N*(sphere.m_Radius - dot_product(vb, tri.N));
		}
		return true;
	}
	return false;
}