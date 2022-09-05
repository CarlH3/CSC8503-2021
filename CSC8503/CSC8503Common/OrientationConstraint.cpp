#include "OrientationConstraint.h"
#include "GameObject.h"
#include "../../Common/Vector3.h"

using namespace NCL;
using namespace CSC8503;
using namespace Maths;

void OrientationConstraint::UpdateConstraint(float dt) {
	Transform& transform = object->GetTransform();
	Quaternion orientation = transform.GetOrientation();
	Vector3 currentDirection = orientation * Vector3(0, 1, 0);

	Vector3 direction = (position - transform.GetPosition()).Normalised();
	float angle = acos(Vector3::Dot(currentDirection, direction));

	if (angle > this->angle) {
		Vector3 rotationAxis = Vector3::Cross(currentDirection, direction).Normalised();

		PhysicsObject* phys = object->GetPhysicsObject();
		float constraintMass = phys->GetInverseMass();

		if (constraintMass > 0.0f) {
			float biasFactor = 0.000001f;
			float bias = -(biasFactor / dt);

			float lambda = -(angle - this->angle + bias) / constraintMass;

			Vector3 impulse = -rotationAxis * lambda;
			phys->ApplyAngularImpulse(impulse);
		}
	}
}