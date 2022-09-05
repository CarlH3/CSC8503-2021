#pragma once
#include "../CSC8503Common/Constraint.h"
#include "../../Common/Vector3.h"

using namespace NCL::Maths;

namespace NCL {
	namespace CSC8503 {
		class GameObject;

		class PositionConstraint : public Constraint {
		public:
			
			PositionConstraint(GameObject* a, GameObject* b, float d) {
				objectA = a;
				objectB = b;
				distance = d;
			}
			
			PositionConstraint(GameObject* a, Vector3 pos, float d) {
				objectA = a;
				objectB = nullptr;
				position = pos;
				distance = d;
			}

			~PositionConstraint() {}

			void UpdateConstraint(float dt) override;

		protected:
			GameObject* objectA;
			GameObject* objectB;
			float distance;
			Vector3 position;
		};
	}
}

