#pragma once
#include "Constraint.h"
#include "../../Common/Vector3.h"
#include "../../Common/Quaternion.h"

using namespace NCL::Maths;

namespace NCL {
	namespace CSC8503 {
		class GameObject;

		class OrientationConstraint : public Constraint {
		public:
			OrientationConstraint(GameObject* o, Vector3 pos, float angle) {
				object = o;
				position = pos;
				this->angle = angle;
			}

			~OrientationConstraint() {}

			void UpdateConstraint(float dt) override;

		protected:
			GameObject* object;
			Vector3 position;
			float angle;
		};
	}
}

