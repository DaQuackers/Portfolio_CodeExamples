#include "Advanced Collisions.h"
// IntersectRayTriangle
//
// In:
//		vert0 - First vertex of the triangle
//		vert1 - Second vertex of the triangle
//		vert2 - Third vertex of the triangle
//		norm - Normal of the triangle
//		start - Start of the ray
//		d - direction of the ray (normalized)
//
// Out:
//		t - Time of intersection, if any
//
// Return:
//		bool - True if intersection, false if not
//
// TODO:
//		If the ray starts behind the triangle or the dot product of the ray normal and tri normal is greater than ED_EPSILON, return false.
//		Implement the algorithm presented in "Intersecting Line to Triangle 2.ppt"
//		Assume that the vertices are already sorted properly.
//	
bool IntersectRayTriangle(const vec3f &vert0, const vec3f &vert1, const vec3f &vert2, const vec3f &norm, const vec3f &start, const vec3f &d, float &t)
{
	// TODO: Read the header file comments for this function!
	if (dot_product(start, norm) < dot_product(vert0, norm) || dot_product(d, norm) > FLT_EPSILON)
		return false;
	vec3f S_[3], N_[3];
	S_[0] = vert0 - start;
	S_[1] = vert1 - start;
	S_[2] = vert2 - start;
	cross_product(N_[0], S_[0], S_[1]);
	cross_product(N_[1], S_[1], S_[2]);
	cross_product(N_[2], S_[2], S_[0]);
	float dot[3];
	dot[0] = dot_product(d, N_[0]); dot[1] = dot_product(d, N_[1]); dot[2] = dot_product(d, N_[2]);
	// TODO: Complete this function
	// Tip: Use the SameSign() macro
	if (SameSign(dot[0], dot[1]) && SameSign(dot[1], dot[2]))
	{
		if (abs(dot[0]) < FLT_EPSILON && abs(dot[1]) < FLT_EPSILON && abs(dot[3]) < FLT_EPSILON)
		{
			t = 0; return true;
		}
		t = (dot_product(vert0, norm) - dot_product(start, norm)) / (dot_product(norm, d));
		return true;
	}
	// *Skip testing against backfacing triangles*
	//	If the ray starts behind the triangle plane OR the angle between ray direction and tri normal is greater than 90 degrees
	//		Stop testing

	return false;
}
// IntersectRaySphere
//
// In:
//		p - start of the ray
//		d - direction of the ray (normalized)
//		center - center point of the sphere
//		radius - radius of the sphere
//
// Out:
//		t - Time of intersection, if any
//		q - point of intersection, if any
//
// Return:
//		bool - True if intersection, false if not
//
// TODO:
//		Implement the algorithm presented in "Advanced Ray to Sphere.ppt"
//		Adjust the algorithm so that the test stops and returns false if the ray points away from the sphere, regardless of ray start position.
//		This adjustment will make it so that a point moving away from or out of the sphere will not intersect.
bool IntersectRaySphere(const vec3f &p, const vec3f &d, const vec3f &center, float radius, float &t, vec3f &q)
{
	// TODO: Read the header file comments for this function!
	vec3f m = p - center;
	if (dot_product(d, m) > FLT_EPSILON)
		return false;
	float b = dot_product(m, d);
	float c = dot_product(m, m) - pow(radius, 2);
	float discr = b*b - c;
	if (discr < FLT_EPSILON)
		return false;
	t = -b - sqrt(discr);
	if (t < FLT_EPSILON) t = 0.0f;
	q = p + d*t;
	return true;
	// TODO: Complete this function
	//		 BE SURE TO MODIFY THE ALGORITHM AS SPECIFIED IN THE HEADER FILE COMMENTS
	return true;
}
//
// In:
//		sa - start of the ray
//		n - direction of the ray (normalized)
//		p - First (base) point on the cylinder segment
//		q - Second (top) point on the cylinder segment
//		r - Radius of the cylinder
//		
// Out:
//		t - Time of intersection, if any
//
// Return:
//		bool - True if intersection, false if not
//
// TODO:
//		Optimization - If the ray starts outside the top or bottom planes and points away, there can be no intersection.
//
//		Use the quadratic formula to determine if and when the ray intersects the cylinder.
//      Components (a,b,c) for the quadratic formula are given in the function body.
//		As with ray to sphere, if the discriminant is less than zero then there is no intersection.
//		If the time of intersection is negative then return no intersection.
//		If the point of intersection is not between the top and bottom planes of the cylinder, return no intersection.
//
//		This test will only detect intersection with the visible section of the cylinder.
//		There is no intersection with the endcaps and no intersection with the backfacing surface.
//
//		For an in depth explanation of a similar algorithm, see 
//		"(5.3.7) Intersecting Ray or Segment Against Cylinder" in "Real-Time Collision Detection"
//		As presented in the book, the algorithm works with finite segments and not rays so it is not all 100% applicable.
//		(Meaning, if you copy from the book you will likely screw yourself)
bool IntersectRayCylinder(const vec3f &sa, const vec3f &n, const vec3f &p, const vec3f &q, float r, float &t)
{
	// TODO: Read the header file comments for this function!

	vec3f d = q - p; // vector from first point on cylinder segment to the end point on cylinder segment
	vec3f m = sa - p; // vector from first point on cylinder segment to start point of ray

	// Values used to calculate coefficients of quadratic formula.
	// You do not necessarily have to use any of these directly for the rest of the algorithm.
	float dd = dot_product(d, d); // dot product of d with d (squared magnitude of d)
	float nd = dot_product(n, d); // dot product of ray normal (n) with d
	float mn = dot_product(m, n);
	float md = dot_product(m, d);
	float mm = dot_product(m, m);

	// TODO: Optimization by early out
	//		 If the ray starts outside the top or bottom planes and points away, there can be no intersection.
	if (dot_product(sa, d) < dot_product(d, p) && dot_product(n, d) < FLT_EPSILON)
		return false;
	if (dot_product(sa, d) > dot_product(d, q) && dot_product(n, d) > FLT_EPSILON)
		return false;

	// Coefficients for the quadratic formula
	float a = dd - nd * nd;
	float b = dd*mn - nd*md;
	float c = dd*(mm - r*r) - md*md;

	// If a is approximately 0.0 then the ray is parallel to the cylinder and can't intersect
	if (abs(a) < FLT_EPSILON)
		return false;

	// TODO: Find time of intersection, if any
	//		 Use the quadratic formula to solve for t. Reference "Advanced Ray to Sphere.ppt" for an example.
	//		 As with "Advanced Ray to Sphere", the 2s and 4 in the formula ( x = (-b - sqrt(b*b - 4ac)) / 2a )
	//		 are cancelled out, resulting in a simplified form.
	float discr = b*b - a*c;
	if (discr < FLT_EPSILON)
		return false;
	t = (-b - sqrt(discr)) / a;
	if (t < FLT_EPSILON)
		return false;
	vec3f inter = sa + n*t;
	if (dot_product(inter, d) < dot_product(d, p))
		return false;
	if (dot_product(inter, d) > dot_product(d, q))
		return false;
	return true;
}
// IntersectRayCapsule
//
//	In:
//		sa - start of the ray
//		n - direction of the ray (normalized)
//		p - First point on the capsule segment
//		q - Second point on the capsule segment
//		r - radius of the capsule
//
//	Out:
//		t - Time of intersection, if any
//
// Return:
//		bool - True if intersection, false if not
//
//	TODO:
//		Determine if/when the ray intersects the cylinder of the capsule.
//			If there is no intersection with the cylinder, test against the spherical endcaps of the capsule finding the earliest time of intersection.

bool IntersectRayCapsule(const vec3f &sa, const vec3f &n, const vec3f &p, const vec3f &q, float r, float &t)
{
	// TODO: Read the header file comments for this function!
	t = FLT_MAX;
	vec3f throwaway;
	if (IntersectRayCylinder(sa, n, p, q, r, t))
		return true;
	bool sphere1, sphere2; float t2;
	sphere1 = IntersectRaySphere(sa, n, p, r, t, throwaway);
	sphere2 = IntersectRaySphere(sa, n, q, r, t2, throwaway);
	if (sphere1 && !sphere2)
		return true;
	if (sphere2 && !sphere1){
		t = t2;
		return true;
	}
	if (sphere1 && sphere2){
		if (t < t2)
			return true;
		t = t2;
		return true;
	}
	// TODO: Complete this function
	return false;
}
vec3f ClosestPointOnLine(vec3f start, vec3f end, vec3f tpt)
{
	vec3f c = end - start;
	vec3f v = tpt - start;
	float d = dot_product(v, c) / dot_product(c, c);
	if (d < FLT_EPSILON) d = 0; if (d > 1) d = 1;
	tpt = start + (c*d);
	return tpt;
}
// IntersectMovingSphereTriangle
//
//	In:
//		vert0 - First vertex of the triangle
//		vert1 - Second vertex of the triangle
//		vert2 - Third vertex of the triangle
//		norm - Normal of the triangle
//		start - Start point of the moving sphere
//		d - direction of the moving sphere (normalized)
//		r - radius of the sphere
//
//	Out:
//		t - Time of intersection, if any
//		outNormal - normal of the surface where the intersection occured, if any
//
// Return:
//		bool - True if intersection, false if not
//
//	TODO:
//		Offset the vertices of the triangle in the direction of the triangle normal by the sphere radius
//		and perform a ray to triangle test. Assume the verts are already properly sorted.
//		If there was an intersection, stop testing and return the triangle normal as outNormal.
//		Else, find the earliest time of intersection (if any) between the ray and the edge capsules of the triangle.
//		(Edge capsules are based off of the original non-offset verts)
//			If there is an edge capsule intersection, calculate the vector from the closest point on the edge to the point of intersection.
//			Normalize this vector and return it as outNormal.
bool IntersectMovingSphereTriangle(const vec3f &vert0, const vec3f &vert1, const vec3f &vert2, const vec3f &norm, const vec3f &start, const vec3f &d, float r, float &t, vec3f &outNormal)
{
	// TODO: Read the header file comments for this function!
	if (dot_product(start, norm) < dot_product(vert0, norm) || dot_product(d, norm) > FLT_EPSILON)
		return false;
	t = FLT_MAX;

	vec3f verts[3] = { vert0, vert1, vert2 };
	for (size_t i = 0; i < 3; i++)
	{
		verts[i] += norm*r;
	}
	if (IntersectRayTriangle(verts[0], verts[1], verts[2], norm, start, d, t)){
		outNormal = norm;
		return true;
	}
	float t1, t2, t3;
	bool cyl1 = IntersectRayCylinder(start, d, vert0, vert1, r, t1);
	bool cyl2 = IntersectRayCylinder(start, d, vert1, vert2, r, t2);
	bool cyl3 = IntersectRayCylinder(start, d, vert2, vert0, r, t3);
	if (cyl1 && !(cyl2 || cyl3)){
		t = t1;
		outNormal = ClosestPointOnLine(vert0, vert1, start) - (start + d*t);
		return true;
	}
	if (cyl2 && !(cyl1 || cyl3)){
		t = t2;
		outNormal = ClosestPointOnLine(vert1, vert2, start) - (start + d*t);
		return true;
	}
	if (cyl3 && !(cyl2 || cyl1)){
		t = t3;
		outNormal = ClosestPointOnLine(vert2, vert0, start) - (start + d*t);
		return true;
	}
	if (cyl1 || cyl2 || cyl3){
		if (t1 < t && cyl1) {
			t = t1;
			outNormal = ClosestPointOnLine(vert0, vert1, start) - (start + d*t);
		}
		if (t2 < t && cyl2){
			t = t2;
			outNormal = ClosestPointOnLine(vert1, vert2, start) - (start + d*t);
		}
		if (t3 < t && cyl3){
			t = t3;
			outNormal = ClosestPointOnLine(vert2, vert0, start) - (start + d*t);
		}
		return true;
	}
	vec3f throw1, throw2, throw3;
	cyl1 = IntersectRaySphere(start, d, vert0, r, t1, throw1);
	cyl2 = IntersectRaySphere(start, d, vert1, r, t2, throw2);
	cyl3 = IntersectRaySphere(start, d, vert2, r, t3, throw3);
	if (cyl1 && !(cyl2 || cyl3)){
		t = t1;
		outNormal = vert0 - throw1;
		return true;
	}
	if (cyl2 && !(cyl1 || cyl3)){
		t = t2;
		outNormal = vert1 - throw2;
		return true;
	}
	if (cyl3 && !(cyl2 || cyl1)){
		t = t3;
		outNormal = vert2 - throw3;
		return true;
	}
	if (cyl1 || cyl2 || cyl3){
		if (t1 < t && cyl1){
			outNormal = vert0 - throw1;
			t = t1;
		}
		if (t2 < t && cyl2){
			outNormal = vert1 - throw2;
			t = t2;
		}
		if (t3 < t && cyl3){
			outNormal = vert2 - throw3;
			t = t3;
		}
		return true;
	}
	// TODO: Complete this function	

	return false;
}
// IntersectMovingSphereMesh
//
//	In:
//		start - Start point of the moving sphere
//		d - direction of the moving sphere (normalized)
//		r - radius of the sphere
//		mesh - pointer to the mesh to perform intersection against
//
//	Out:
//		t - Time of intersection, if any
//		outNormal - normal of the surface where the intersection occured, if any
//
// Return:
//		bool - True if intersection, false if not
//
//	TODO:
//		For each triangle...
//			Sort the indices from lowest to highest as described in "Intersecting Line to Triangle 2.ppt".
//				This sorting can be done in a handful of "if" statements. You are sorting a very small set of values, keep it simple!
//			Perform IntersectMovingSphereTriangle with each triangle, finding the earliest time of intersection.
//			Return the surface normal at the point of earliest intersection as outNormal.
bool IntersectMovingSphereMesh(const vec3f &start, const vec3f &d, float r, const ED2Mesh* mesh, float &t, vec3f &outNormal)
{
	// TODO: Read the header file comments for this function!

	bool bCollision = false;
	t = FLT_MAX;
	float t2 = FLT_MAX;
	size_t ind[3];
	vec3f temp;
	for (size_t i = 0; i < mesh->m_Triangles.size(); i++)
	{
		ind[0] = mesh->m_Triangles[i].indices[0];
		ind[1] = mesh->m_Triangles[i].indices[1];
		ind[2] = mesh->m_Triangles[i].indices[2];
		//butterfly swap sort;
		if (ind[0] > ind[1]){
			ind[0] ^= ind[1]; ind[1] ^= ind[0]; ind[0] ^= ind[1];
		}
		if (ind[1] > ind[2]) {
			ind[1] ^= ind[2]; ind[2] ^= ind[1]; ind[1] ^= ind[2];
		}
		if (ind[0] > ind[1]){
			ind[0] ^= ind[1]; ind[1] ^= ind[0]; ind[0] ^= ind[1];
		}
		ED2Vertex vert0 = mesh->m_Vertices[ind[0]], vert1 = mesh->m_Vertices[ind[1]], vert2 = mesh->m_Vertices[ind[2]];
		//test collision
		if (IntersectMovingSphereTriangle(vert0.pos, vert1.pos, vert2.pos, mesh->m_TriNorms[i], start, d, r, t2, temp))
		{
			bCollision = true;
			if (t2 < t){
				t = t2;
				outNormal = temp;
			}
		}
	}
	// TODO: Complete this function

	return bCollision;
}



