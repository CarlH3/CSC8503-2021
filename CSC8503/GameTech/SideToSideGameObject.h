#pragma once
#include "../CSC8503Common/StateGameObject.h"
#include "../CSC8503Common/StateMachine.h"

namespace NCL {
	namespace CSC8503 {
		class SideToSideGameObject : public StateGameObject {
		public:
			SideToSideGameObject(float forceMagnitude, std::string name = "");
			~SideToSideGameObject() {}

		protected:
			void MoveLeft(float dt);
			void MoveRight(float dt);

			float forceMagnitude;
		};
	}
}