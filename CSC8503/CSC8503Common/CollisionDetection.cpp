#include "CollisionDetection.h"
#include "CollisionVolume.h"
#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "../../Common/Vector2.h"
#include "../../Common/Window.h"
#include "../../Common/Maths.h"
#include "Debug.h"

#include <list>

using namespace NCL;

bool CollisionDetection::RayPlaneIntersection(const Ray&r, const Plane&p, RayCollision& collisions) {
	float ln = Vector3::Dot(p.GetNormal(), r.GetDirection());

	if (ln == 0.0f) {
		return false; //direction vectors are perpendicular!
	}
	
	Vector3 planePoint = p.GetPointOnPlane();

	Vector3 pointDir = planePoint - r.GetPosition();

	float d = Vector3::Dot(pointDir, p.GetNormal()) / ln;

	collisions.collidedAt = r.GetPosition() + (r.GetDirection() * d);

	return true;
}

bool CollisionDetection::RayIntersection(const Ray& r,GameObject& object, RayCollision& collision) {
	bool hasCollided = false;

	const Transform& worldTransform = object.GetTransform();
	const CollisionVolume* volume	= object.GetBoundingVolume();

	if (!volume) {
		return false;
	}

	switch (volume->type) {
		case VolumeType::AABB:		hasCollided = RayAABBIntersection(r, worldTransform, (const AABBVolume&)*volume	, collision); break;
		case VolumeType::OBB:		hasCollided = RayOBBIntersection(r, worldTransform, (const OBBVolume&)*volume	, collision); break;
		case VolumeType::Sphere:	hasCollided = RaySphereIntersection(r, worldTransform, (const SphereVolume&)*volume	, collision); break;
		case VolumeType::Capsule:    hasCollided = RayCapsuleIntersection(r, worldTransform, (const CapsuleVolume&)*volume, collision); break;
	}

	return hasCollided;
}

bool CollisionDetection::RayBoxIntersection(const Ray&r, const Vector3& boxPos, const Vector3& boxSize, RayCollision& collision) {
	Vector3 boxMin = boxPos - boxSize;
	Vector3 boxMax = boxPos + boxSize;

	Vector3 rayPos = r.GetPosition();
	Vector3 rayDir = r.GetDirection();

	Vector3 tVals(-1, -1, -1);

	for (int i = 0; i < 3; ++i) {
		if (rayDir[i] > 0)
			tVals[i] = (boxMin[i] - rayPos[i]) / rayDir[i];
		else if (rayDir[i] < 0)
			tVals[i] = (boxMax[i] - rayPos[i]) / rayDir[i];
	}

	float bestT = tVals.GetMaxElement();
	
	if (bestT < 0.0f)
		return false;

	Vector3 intersection = rayPos + (rayDir * bestT);
	const float epsilon = 0.0001f; //an amount of leeway in our calcs

	for (int i = 0; i < 3; ++i) {
		if (intersection[i] + epsilon < boxMin[i] || intersection[i] - epsilon > boxMax[i])
			return false; //best intersection doesn’t touch the box!
	}

	collision.collidedAt = intersection;
	collision.rayDistance = bestT;

	return true;
}

bool CollisionDetection::RayAABBIntersection(const Ray&r, const Transform& worldTransform, const AABBVolume& volume, RayCollision& collision) {
	Vector3 boxPos = worldTransform.GetPosition();
	Vector3 boxSize = volume.GetHalfDimensions();

	return RayBoxIntersection(r, boxPos, boxSize, collision);
}

bool CollisionDetection::RayOBBIntersection(const Ray&r, const Transform& worldTransform, const OBBVolume& volume, RayCollision& collision) {
	Quaternion orientation = worldTransform.GetOrientation();
	Vector3 position = worldTransform.GetPosition();

	Matrix3 transform = Matrix3(orientation);
	Matrix3 invTransform = Matrix3(orientation.Conjugate());

	Vector3 localRayPos = r.GetPosition() - position;

	Ray tempRay(invTransform * localRayPos, invTransform * r.GetDirection());

	bool collided = RayBoxIntersection(tempRay, Vector3(), volume.GetHalfDimensions(), collision);

	if (collided)
		collision.collidedAt = transform * collision.collidedAt + position;

	return collided;
}

bool CollisionDetection::RayCapsuleIntersection(const Ray& r, const Transform& worldTransform, const CapsuleVolume& volume, RayCollision& collision) {
	// Half height used is that of cyclinder section rather than that of whole capsule
	Vector3 capsuleUp = (worldTransform.GetOrientation() * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	Vector3 capsulePos = worldTransform.GetPosition();
	float capsuleRadius = volume.GetRadius();
	float capsuleHalfHeight = volume.GetHalfHeight();
	Vector3 endPoint1 = capsulePos + (capsuleUp * capsuleHalfHeight);
	Vector3 endPoint2 = capsulePos - (capsuleUp * capsuleHalfHeight);

	Vector3 a = endPoint1 - capsulePos;
	Vector3 b = r.GetPosition() - capsulePos;
	Vector3 aCrossB = Vector3::Cross(a, b);
	Vector3 thirdPoint = capsulePos + aCrossB;

	Vector3 planeNormal = Vector3::Cross(aCrossB, a).Normalised();

	Plane plane = Plane(planeNormal, -Vector3::Dot(planeNormal, capsulePos));
	RayCollision planeCollision;
	RayPlaneIntersection(r, plane, planeCollision);

	Vector3 p = planeCollision.collidedAt;

	Vector3 centreLine = endPoint2 - endPoint1;
	float t = Clamp(Vector3::Dot(p - endPoint1, centreLine) / Vector3::Dot(centreLine, centreLine), 0.0f, 1.0f);
	Vector3 closestPointOnLine = endPoint1 + (centreLine * t);

	if ((p - closestPointOnLine).Length() < capsuleRadius) {
		Transform t = Transform();
		t.SetPosition(closestPointOnLine);

		SphereVolume s = SphereVolume(capsuleRadius);

		return RaySphereIntersection(r, t, s, collision);
	}

	return false;
}

bool CollisionDetection::RaySphereIntersection(const Ray& r, const Transform& worldTransform, const SphereVolume& volume, RayCollision& collision) {
	Vector3 spherePos = worldTransform.GetPosition();
	float sphereRadius = volume.GetRadius();

	//Get the direction between the ray origin and the sphere origin
	Vector3 dir = (spherePos - r.GetPosition());

	//Then project the sphere ’s origin onto our ray direction vector
	float sphereProj = Vector3::Dot(dir, r.GetDirection());

	if (sphereProj < 0.0f) {
		return false; // point is behind the ray!
	}

	//Get closest point on ray line to sphere
	Vector3 point = r.GetPosition() + (r.GetDirection() * sphereProj);

	float sphereDist = (point - spherePos).Length();

	if (sphereDist > sphereRadius) {
		return false;
	}
	float offset = sqrt((sphereRadius * sphereRadius) - (sphereDist * sphereDist));

	collision.rayDistance = sphereProj - (offset);
	collision.collidedAt = r.GetPosition() + (r.GetDirection() * collision.rayDistance);

	return true;
}

Matrix4 GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix4::Translation(position) *
		Matrix4::Rotation(-yaw, Vector3(0, -1, 0)) *
		Matrix4::Rotation(-pitch, Vector3(-1, 0, 0));

	return iview;
}

Vector3 CollisionDetection::Unproject(const Vector3& screenPos, const Camera& cam) {
	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	float aspect	= screenSize.x / screenSize.y;
	float fov		= cam.GetFieldOfVision();
	float nearPlane = cam.GetNearPlane();
	float farPlane  = cam.GetFarPlane();

	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(cam) * GenerateInverseProjection(aspect, fov, nearPlane, farPlane);

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(screenPos.x / (float)screenSize.x) * 2.0f - 1.0f,
		(screenPos.y / (float)screenSize.y) * 2.0f - 1.0f,
		(screenPos.z),
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

Ray CollisionDetection::BuildRayFromMouse(const Camera& cam) {
	Vector2 screenMouse = Window::GetMouse()->GetAbsolutePosition();
	Vector2 screenSize	= Window::GetWindow()->GetScreenSize();

	//We remove the y axis mouse position from height as OpenGL is 'upside down',
	//and thinks the bottom left is the origin, instead of the top left!
	Vector3 nearPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		-0.99999f
	);

	//We also don't use exactly 1.0 (the normalised 'end' of the far plane) as this
	//causes the unproject function to go a bit weird. 
	Vector3 farPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		0.99999f
	);

	Vector3 a = Unproject(nearPos, cam);
	Vector3 b = Unproject(farPos, cam);
	Vector3 c = b - a;

	c.Normalise();

	//std::cout << "Ray Direction:" << c << std::endl;

	return Ray(cam.GetPosition(), c);
}

//http://bookofhook.com/mousepick.pdf
Matrix4 CollisionDetection::GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	Matrix4 m;

	float t = tan(fov*PI_OVER_360);

	float neg_depth = nearPlane - farPlane;

	const float h = 1.0f / t;

	float c = (farPlane + nearPlane) / neg_depth;
	float e = -1.0f;
	float d = 2.0f*(nearPlane*farPlane) / neg_depth;

	m.array[0]  = aspect / h;
	m.array[5]  = tan(fov*PI_OVER_360);

	m.array[10] = 0.0f;
	m.array[11] = 1.0f / d;

	m.array[14] = 1.0f / e;

	m.array[15] = -c / (d*e);

	return m;
}

/*
And here's how we generate an inverse view matrix. It's pretty much
an exact inversion of the BuildViewMatrix function of the Camera class!
*/
Matrix4 CollisionDetection::GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
Matrix4::Translation(position) *
Matrix4::Rotation(yaw, Vector3(0, 1, 0)) *
Matrix4::Rotation(pitch, Vector3(1, 0, 0));

return iview;
}


/*
If you've read through the Deferred Rendering tutorial you should have a pretty
good idea what this function does. It takes a 2D position, such as the mouse
position, and 'unprojects' it, to generate a 3D world space position for it.

Just as we turn a world space position into a clip space position by multiplying
it by the model, view, and projection matrices, we can turn a clip space
position back to a 3D position by multiply it by the INVERSE of the
view projection matrix (the model matrix has already been assumed to have
'transformed' the 2D point). As has been mentioned a few times, inverting a
matrix is not a nice operation, either to understand or code. But! We can cheat
the inversion process again, just like we do when we create a view matrix using
the camera.

So, to form the inverted matrix, we need the aspect and fov used to create the
projection matrix of our scene, and the camera used to form the view matrix.

*/
Vector3	CollisionDetection::UnprojectScreenPosition(Vector3 position, float aspect, float fov, const Camera &c) {
	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(c) * GenerateInverseProjection(aspect, fov, c.GetNearPlane(), c.GetFarPlane());

	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(position.x / (float)screenSize.x) * 2.0f - 1.0f,
		(position.y / (float)screenSize.y) * 2.0f - 1.0f,
		(position.z) - 1.0f,
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}


bool CollisionDetection::ObjectIntersection(GameObject* a, GameObject* b, CollisionInfo& collisionInfo) {
	const CollisionVolume* volA = a->GetBoundingVolume();
	const CollisionVolume* volB = b->GetBoundingVolume();

	if (!volA || !volB) {
		return false;
	}

	collisionInfo.a = a;
	collisionInfo.b = b;

	Transform& transformA = a->GetTransform();
	Transform& transformB = b->GetTransform();

	VolumeType pairType = (VolumeType)((int)volA->type | (int)volB->type);

	if (pairType == VolumeType::AABB) {
		return AABBIntersection((AABBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::Sphere) {
		return SphereIntersection((SphereVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::OBB) {
		return OBBIntersection((OBBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::Capsule) {
		return CapsuleIntersection((CapsuleVolume&)*volA, transformA, (CapsuleVolume&)*volB, transformB, collisionInfo);
	}
	//capsule vs AABB 
	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Capsule) {
		return AABBCapsuleIntersection((AABBVolume&)*volA, transformA, (CapsuleVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBCapsuleIntersection((AABBVolume&)*volB, transformB, (CapsuleVolume&)*volA, transformA, collisionInfo);
	}

	//AABB VS Sphere
	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Sphere) {
		return AABBSphereIntersection((AABBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBSphereIntersection((AABBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}
	//Capsule VS Sphere
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Sphere) {
		return SphereCapsuleIntersection((CapsuleVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::Capsule) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return SphereCapsuleIntersection((CapsuleVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}
	// OBB VS Capsule
	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Capsule) {
		return OBBCapsuleIntersection((OBBVolume&)*volA, transformA, (CapsuleVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBCapsuleIntersection((OBBVolume&)*volB, transformB, (CapsuleVolume&)*volA, transformA, collisionInfo);
	}
	//OBB VS SPhere
	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Sphere) {
		return OBBSphereIntersection((OBBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBSphereIntersection((OBBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}
	//Obb Vs AABB
	if (volA->type == VolumeType::OBB && volB->type == VolumeType::AABB) {
		return OBBAABBIntersection((OBBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::AABB && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBAABBIntersection((OBBVolume&)*volB, transformB, (AABBVolume&)*volA, transformA, collisionInfo);
	}

	return false;
}
//Plane Sphere
bool CollisionDetection::PlaneSphereIntersection(const Plane& plane, const SphereVolume& volume, const Transform& worldTransform) {
	return abs(Vector3::Dot(worldTransform.GetPosition(), plane.GetNormal()) + plane.GetDistance()) <= volume.GetRadius();
}
//Plane AABB
bool CollisionDetection::PlaneAABBIntersection(const Plane& plane, const AABBVolume& volume, const Transform& worldTransform) {
	Vector3 halfDim = volume.GetHalfDimensions();
	Vector3 planeNor = plane.GetNormal();

	float radius = halfDim.x * abs(planeNor.x) + halfDim.y * abs(planeNor.y) + halfDim.z * abs(planeNor.z);
	float distance = abs(Vector3::Dot(worldTransform.GetPosition(), planeNor) + plane.GetDistance());

	return distance <= radius;
}

bool CollisionDetection::PlaneCapsuleIntersection(const Plane& plane, const CapsuleVolume& volume, const Transform& worldTransform) {
	// Half height used is that of cyclinder section rather than that of whole capsule
	Vector3 capsuleUp = (worldTransform.GetOrientation() * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	Vector3 endPoint1 = worldTransform.GetPosition() + (capsuleUp * volume.GetHalfHeight());
	Vector3 endPoint2 = worldTransform.GetPosition() - (capsuleUp * volume.GetHalfHeight());

	if (abs(Vector3::Dot(endPoint1, plane.GetNormal()) + plane.GetDistance()) <= volume.GetRadius()) {
		return true;
	}

	if (abs(Vector3::Dot(endPoint2, plane.GetNormal()) + plane.GetDistance()) <= volume.GetRadius()) {
		return true;
	}

	return false;
}

bool CollisionDetection::AABBTest(const Vector3& posA, const Vector3& posB,
	const Vector3& halfSizeA, const Vector3& halfSizeB) {

	Vector3 delta = posB - posA;
	Vector3 totalSize = halfSizeA + halfSizeB;

	if (abs(delta.x) < totalSize.x && abs(delta.y) < totalSize.y && abs(delta.z) < totalSize.z)
		return true;

	return false;
}

//AABB/AABB Collisions
bool CollisionDetection::AABBIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxAPos = worldTransformA.GetPosition();
	Vector3 boxBPos = worldTransformB.GetPosition();

	Vector3 boxASize = volumeA.GetHalfDimensions();
	Vector3 boxBSize = volumeB.GetHalfDimensions();

	bool overlap = AABBTest(boxAPos, boxBPos, boxASize, boxBSize);

	if (overlap) {
		static const Vector3 faces[6] = {
			Vector3(-1, 0, 0), Vector3(1, 0, 0),
			Vector3(0, -1, 0), Vector3(0, 1, 0),
			Vector3(0, 0, -1), Vector3(0, 0, 1),
		};
		
		Vector3 maxA = boxAPos + boxASize;
		Vector3 minA = boxAPos - boxASize;

		Vector3 maxB = boxBPos + boxBSize;
		Vector3 minB = boxBPos - boxBSize;

		float distances[6] = {
			(maxB.x - minA.x),// distance of box ¡¯b¡¯ to ¡¯left¡¯ of ¡¯a¡¯.
			(maxA.x - minB.x),// distance of box ¡¯b¡¯ to ¡¯right¡¯ of ¡¯a¡¯.
			(maxB.y - minA.y),// distance of box ¡¯b¡¯ to ¡¯bottom ¡¯ of ¡¯a¡¯.
			(maxA.y - minB.y),// distance of box ¡¯b¡¯ to ¡¯top¡¯ of ¡¯a¡¯.
			(maxB.z - minA.z),// distance of box ¡¯b¡¯ to ¡¯far¡¯ of ¡¯a¡¯.
			(maxA.z - minB.z) // distance of box ¡¯b¡¯ to ¡¯near¡¯ of ¡¯a¡¯.
		};

		float penetration = FLT_MAX;
		Vector3 bestAxis;

		for (int i = 0; i < 6; i++) {
			if (distances[i] < penetration) {
				penetration = distances[i];
				bestAxis = faces[i];
			}
		}
		collisionInfo.AddContactPoint(Vector3(), Vector3(), bestAxis, penetration);

		return true;
	}

	return false;
}

//Sphere / Sphere Collision
bool CollisionDetection::SphereIntersection(const SphereVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float radii = volumeA.GetRadius() + volumeB.GetRadius();
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	const float deltaLength = delta.Length();

	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = delta.Normalised();
		Vector3 localA = normal * volumeA.GetRadius();
		Vector3 localB = -normal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		return true;//we¡¯re colliding!
	}

	return false;
}

//AABB - Sphere Collision
bool CollisionDetection::AABBSphereIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxSize = volumeA.GetHalfDimensions();
	
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();
	
	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

	Vector3 localPoint = delta - closestPointOnBox;
	float distance = localPoint.Length();
	
	if (distance < volumeB.GetRadius()) {//yes , we¡¯re colliding!
		Vector3 collisionNor = localPoint.Normalised();
		float penetration = (volumeB.GetRadius() - distance);

		Vector3 localA = Vector3();//
		Vector3 localB = -collisionNor * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNor, penetration);
		return true;
	}

	return false;
}

//Capsule
bool CollisionDetection::CapsuleIntersection(const CapsuleVolume& volumeA, const Transform& worldTransformA, const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	
	Vector3 capsuleUpA = (worldTransformA.GetOrientation() * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	Vector3 capsuleUpB = (worldTransformB.GetOrientation() * Vector3(0.0f, 1.0f, 0.0f)).Normalised();

	Vector3 endPointA1 = worldTransformA.GetPosition() + (capsuleUpA * volumeA.GetHalfHeight());
	Vector3 endPointA2 = worldTransformA.GetPosition() - (capsuleUpA * volumeA.GetHalfHeight());

	Vector3 endPointB1 = worldTransformB.GetPosition() + (capsuleUpB * volumeB.GetHalfHeight());
	Vector3 endPointB2 = worldTransformB.GetPosition() - (capsuleUpB * volumeB.GetHalfHeight());

	Vector3 pointA;

	float dist1 = (endPointB1 - endPointA1).Length();
	float dist2 = (endPointB2 - endPointA1).Length();

	float dist3 = (endPointB1 - endPointA2).Length();
	float dist4 = (endPointB2 - endPointA2).Length();


	if (dist3 < dist1 || dist3 < dist2 || dist4 < dist1 || dist4 < dist2)
	{
		pointA = endPointA2;
	}
	else 
	{
		pointA = endPointA1;
	}

	Vector3 centreLineB = endPointB2 - endPointB1;
	float t1 = Clamp(Vector3::Dot(pointA - endPointB1, centreLineB) / Vector3::Dot(centreLineB, centreLineB), 0.0f, 1.0f);
	Vector3 closestPointOnLineB = endPointB1 + (centreLineB * t1);

	Vector3 centreLineA = endPointA2 - endPointA1;
	float t2 = Clamp(Vector3::Dot(closestPointOnLineB - endPointA1, centreLineA) / Vector3::Dot(centreLineA, centreLineA), 0.0f, 1.0f);
	Vector3 closestPointOnLineA = endPointA1 + (centreLineA * t2);

	Vector3 delta = closestPointOnLineB - closestPointOnLineA;

	float deltaLength = delta.Length();
	float radii = volumeA.GetRadius() + volumeB.GetRadius();

	if (deltaLength < radii) {
		Vector3 collisionNor = delta.Normalised();
		float penetration = radii - deltaLength;
		Vector3 localA = collisionNor * volumeA.GetRadius();
		Vector3 localB = -collisionNor * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNor, penetration);

		return true;
	}

	return false;
}
//AABB Capsule
bool CollisionDetection::AABBCapsuleIntersection(const AABBVolume& volumeA, const Transform& worldTransformA, const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	
	Vector3 capsuleUp = (worldTransformB.GetOrientation() * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	Vector3 endPoint1 = worldTransformB.GetPosition() + (capsuleUp * volumeB.GetHalfHeight());
	Vector3 endPoint2 = worldTransformB.GetPosition() - (capsuleUp * volumeB.GetHalfHeight());

	Vector3 centreLine = endPoint2 - endPoint1;
	float t = Clamp(Vector3::Dot(worldTransformA.GetPosition() - endPoint1, centreLine) / Vector3::Dot(centreLine, centreLine), 0.0f, 1.0f);
	Vector3 closestPointOnLine = endPoint1 + (centreLine * t);

	Vector3 boxSize = volumeA.GetHalfDimensions();
	Vector3 delta = closestPointOnLine - worldTransformA.GetPosition();
	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);
	Vector3 localPoint = delta - closestPointOnBox;
	float distance = localPoint.Length();

	Vector3 delta1 = endPoint1 - worldTransformA.GetPosition();
	Vector3 closestPointOnBox1 = Maths::Clamp(delta1, -boxSize, boxSize);
	Vector3 localPoint1 = delta1 - closestPointOnBox1;
	float distance1 = localPoint1.Length();

	Vector3 delta2 = endPoint2 - worldTransformA.GetPosition();
	Vector3 closestPointOnBox2 = Maths::Clamp(delta2, -boxSize, boxSize);
	Vector3 localPoint2 = delta2 - closestPointOnBox2;
	float distance2 = localPoint2.Length();

	if (distance1 < distance) {
		distance = distance1;
		localPoint = localPoint1;
	}

	if (distance2 < distance) {
		distance = distance2;
		localPoint = localPoint2;
	}

	if (distance < volumeB.GetRadius()) {
		Vector3 collisionNor = localPoint.Normalised();
		float penetration = volumeB.GetRadius() - distance;

		Vector3 localA = Vector3();
		Vector3 localB = -collisionNor * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNor, penetration);

		return true;
	}

	return false;
}
//Sphere Capsule
bool CollisionDetection::SphereCapsuleIntersection(const CapsuleVolume& volumeA, const Transform& worldTransformA, const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo)
{

	Vector3 capsuleUp = (worldTransformA.GetOrientation() * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	Vector3 endPoint1 = worldTransformA.GetPosition() + (capsuleUp * volumeA.GetHalfHeight());
	Vector3 endPoint2 = worldTransformA.GetPosition() - (capsuleUp * volumeA.GetHalfHeight());

	Vector3 centreLine = endPoint2 - endPoint1;
	float t = Clamp(Vector3::Dot(worldTransformB.GetPosition() - endPoint1, centreLine) / Vector3::Dot(centreLine, centreLine), 0.0f, 1.0f);
	Vector3 closestPointOnLine = endPoint1 + (centreLine * t);

	Vector3 delta = worldTransformB.GetPosition() - closestPointOnLine;
	float deltaLength = delta.Length();
	float radii = volumeA.GetRadius() + volumeB.GetRadius();

	if (deltaLength < radii) {
		Vector3 collisionNor = delta.Normalised();
		float penetration = radii - deltaLength;
		Vector3 localA = collisionNor * volumeA.GetRadius();
		Vector3 localB = -collisionNor * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNor, penetration);

		return true;
	}

	return false;
}
//OBB Capsule
bool CollisionDetection::OBBCapsuleIntersection(const OBBVolume& volumeA, const Transform& worldTransformA, const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Quaternion obbOri = worldTransformA.GetOrientation();
	Quaternion invOBBOri = obbOri.Conjugate();

	Vector3 obbPos = invOBBOri * worldTransformA.GetPosition();
	Vector3 capsulePos = invOBBOri * worldTransformB.GetPosition();

	Vector3 capsuleUp = (worldTransformB.GetOrientation() * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	capsuleUp = (invOBBOri * capsuleUp).Normalised();
	Vector3 endPoint1 = capsulePos + (capsuleUp * volumeB.GetHalfHeight());
	Vector3 endPoint2 = capsulePos - (capsuleUp * volumeB.GetHalfHeight());

	Vector3 centreLine = endPoint2 - endPoint1;
	float t = Clamp(Vector3::Dot(obbPos - endPoint1, centreLine) / Vector3::Dot(centreLine, centreLine), 0.0f, 1.0f);
	Vector3 closestPointOnLine = endPoint1 + (centreLine * t);

	Vector3 boxSize = volumeA.GetHalfDimensions();

	Vector3 delta = closestPointOnLine - obbPos;
	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);
	Vector3 localPoint = delta - closestPointOnBox;
	float distance = localPoint.Length();

	Vector3 delta1 = endPoint1 - worldTransformA.GetPosition();
	Vector3 closestPointOnBox1 = Maths::Clamp(delta1, -boxSize, boxSize);
	Vector3 localPoint1 = delta1 - closestPointOnBox1;
	float distance1 = localPoint1.Length();

	Vector3 delta2 = endPoint2 - worldTransformA.GetPosition();
	Vector3 closestPointOnBox2 = Maths::Clamp(delta2, -boxSize, boxSize);
	Vector3 localPoint2 = delta2 - closestPointOnBox2;
	float distance2 = localPoint2.Length();

	if (distance1 < distance) {
		distance = distance1;
		localPoint = localPoint1;
	}

	if (distance2 < distance) {
		distance = distance2;
		localPoint = localPoint2;
	}

	if (distance < volumeB.GetRadius()) {
		Vector3 collisionNor = obbOri * localPoint.Normalised();
		float penetration = volumeB.GetRadius() - distance;

		Vector3 localA = obbOri * closestPointOnBox;
		Vector3 localB = obbOri * (-collisionNor * volumeB.GetRadius());

		collisionInfo.AddContactPoint(localA, localB, collisionNor, penetration);

		return true;
	}

	return false;
}

//OBB
bool CollisionDetection::OBBIntersection(const OBBVolume& volumeA, const Transform& worldTransformA, const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Quaternion oriA = worldTransformA.GetOrientation();
	Quaternion oriB = worldTransformB.GetOrientation();

	Vector3 axes[15];

	axes[0] = (oriA * Vector3(1.0f, 0.0f, 0.0f)).Normalised();
	axes[1] = (oriA * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	axes[2] = (oriA * Vector3(0.0f, 0.0f, 1.0f)).Normalised();

	axes[3] = (oriB * Vector3(1.0f, 0.0f, 0.0f)).Normalised();
	axes[4] = (oriB * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	axes[5] = (oriB * Vector3(0.0f, 0.0f, 1.0f)).Normalised();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			axes[(i * 3) + j + 6] = Vector3::Cross(axes[i], axes[j + 3]).Normalised();
		}
	}

	Vector3 supportPoints[4];
	float leastPenetration = FLT_MAX;
	Vector3 collisionNor;
	Vector3 localA;
	Vector3 localB;

	for (int i = 0; i < 15; i++) {
		if (axes[i] != Vector3(0, 0, 0)) {
			supportPoints[0] = OBBSupport(worldTransformA, axes[i]);
			supportPoints[1] = OBBSupport(worldTransformA, -axes[i]);

			supportPoints[2] = OBBSupport(worldTransformB, axes[i]);
			supportPoints[3] = OBBSupport(worldTransformB, -axes[i]);

			float maxA = Vector3::Dot(supportPoints[0], axes[i]);
			float minA = Vector3::Dot(supportPoints[1], axes[i]);

			float maxB = Vector3::Dot(supportPoints[2], axes[i]);
			float minB = Vector3::Dot(supportPoints[3], axes[i]);


			float penetration;
			Vector3 pointA;
			Vector3 pointB;

			if (minA <= minB && maxA >= minB) {
				penetration = maxA - minB;
				pointA = supportPoints[0];
				pointB = supportPoints[3];
			}
			else if (minB <= minA && maxB >= minA) {
				penetration = maxB - minA;
				axes[i] = -axes[i];
				pointA = supportPoints[1];
				pointB = supportPoints[2];
			}
			else {
				return false;
			}

			penetration = abs(penetration);

			if (penetration < leastPenetration) {
				leastPenetration = penetration;
				collisionNor = axes[i];

				localA = pointA;
				localB = pointB;

				if ((worldTransformA.GetPosition() - localB).Length() < (worldTransformA.GetPosition() - localA).Length()) {
					localA = localB;
				}
				else if ((worldTransformB.GetPosition() - localA).Length() < (worldTransformB.GetPosition() - localB).Length()) {
					localB = localA;
				}
			}
		}
	}

	localA -= worldTransformA.GetPosition();
	localB -= worldTransformB.GetPosition();

	collisionInfo.AddContactPoint(localA, localB, collisionNor, leastPenetration);

	return true;
}
//OBB AABB
bool CollisionDetection::OBBAABBIntersection(const OBBVolume& volumeA, const Transform& worldTransformA, const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Quaternion oriA = worldTransformA.GetOrientation();

	Vector3 axes[15];

	axes[0] = (oriA * Vector3(1.0f, 0.0f, 0.0f)).Normalised();
	axes[1] = (oriA * Vector3(0.0f, 1.0f, 0.0f)).Normalised();
	axes[2] = (oriA * Vector3(0.0f, 0.0f, 1.0f)).Normalised();

	axes[3] = Vector3(1.0f, 0.0f, 0.0f).Normalised();
	axes[4] = Vector3(0.0f, 1.0f, 0.0f).Normalised();
	axes[5] = Vector3(0.0f, 0.0f, 1.0f).Normalised();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			axes[(i * 3) + j + 6] = Vector3::Cross(axes[i], axes[j + 3]).Normalised();
		}
	}

	float leastPenetration = FLT_MAX;

	Vector3 collisionNor;
	Vector3 localA;
	Vector3 localB;

	for (int i = 0; i < 15; i++) {
		if (axes[i] != Vector3(0, 0, 0)) {
			Vector3 supportPoints[4];

			supportPoints[0] = OBBSupport(worldTransformA, axes[i]);
			supportPoints[1] = OBBSupport(worldTransformA, -axes[i]);

			supportPoints[2] = AABBSupport(worldTransformB, axes[i]);
			supportPoints[3] = AABBSupport(worldTransformB, -axes[i]);

			float maxA = Vector3::Dot(supportPoints[0], axes[i]);
			float minA = Vector3::Dot(supportPoints[1], axes[i]);

			float maxB = Vector3::Dot(supportPoints[2], axes[i]);
			float minB = Vector3::Dot(supportPoints[3], axes[i]);

			float penetration;
			Vector3 pointA;
			Vector3 pointB;

			if (minA <= minB && maxA >= minB) {
				penetration = maxA - minB;
				pointA = supportPoints[0];
				pointB = supportPoints[3];
			}
			else if (minB <= minA && maxB >= minA) {
				penetration = maxB - minA;
				axes[i] = -axes[i];
				pointA = supportPoints[1];
				pointB = supportPoints[2];
			}
			else {
				return false;
			}

			penetration = abs(penetration);

			if (penetration < leastPenetration) {
				leastPenetration = penetration;
				collisionNor = axes[i];

				localA = pointA;
				localB = pointB;

				if ((worldTransformA.GetPosition() - localB).Length() < (worldTransformA.GetPosition() - localA).Length()) {
					localA = localB;
				}
				else if ((worldTransformB.GetPosition() - localA).Length() < (worldTransformB.GetPosition() - localB).Length()) {
					localB = localA;
				}
			}
		}
	}

	localA -= worldTransformA.GetPosition();
	localB = Vector3();

	collisionInfo.AddContactPoint(localA, localB, collisionNor, leastPenetration);

	return true;
}
//OBB Sphere
bool CollisionDetection::OBBSphereIntersection(const OBBVolume& volumeA, const Transform& worldTransformA, const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Quaternion obbOri = worldTransformA.GetOrientation();
	Quaternion invOBBOri = obbOri.Conjugate();

	Vector3 obbPos = invOBBOri * worldTransformA.GetPosition();
	Vector3 spherePos = invOBBOri * worldTransformB.GetPosition();

	Vector3 boxSize = volumeA.GetHalfDimensions();

	Vector3 delta = spherePos - obbPos;

	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);
	Vector3 localPoint = delta - closestPointOnBox;
	float distance = localPoint.Length();

	if (distance < volumeB.GetRadius()) {
		Vector3 collisionNor = obbOri * localPoint.Normalised();
		float penetration = volumeB.GetRadius() - distance;

		Vector3 localA = obbOri * closestPointOnBox;
		Vector3 localB = obbOri * (-collisionNor * volumeB.GetRadius());

		collisionInfo.AddContactPoint(localA, localB, collisionNor, penetration);

		return true;
	}

	return false;
}

Vector3 CollisionDetection::AABBSupport(const Transform& worldTransform, Vector3 worldDir) {
	Vector3 vertex;

	vertex.x = worldDir.x < 0 ? -0.5f : 0.5f;
	vertex.y = worldDir.y < 0 ? -0.5f : 0.5f;
	vertex.z = worldDir.z < 0 ? -0.5f : 0.5f;

	return worldTransform.GetMatrix() * vertex;
}

Vector3 CollisionDetection::OBBSupport(const Transform& worldTransform, Vector3 worldDir) {
	Vector3 localDir = worldTransform.GetOrientation().Conjugate() * worldDir;
	Vector3 vertex;

	vertex.x = localDir.x < 0 ? -0.5f : 0.5f;
	vertex.y = localDir.y < 0 ? -0.5f : 0.5f;
	vertex.z = localDir.z < 0 ? -0.5f : 0.5f;

	return worldTransform.GetMatrix() * vertex;
}