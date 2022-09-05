#pragma once
#include "../CSC8503Common/GameObject.h"

namespace NCL {
	namespace CSC8503 {
		class AIGameObject : public GameObject {
		public:
			AIGameObject(std::string name) : GameObject(name) {}
			~AIGameObject() {}

			virtual void UpdateAI(float dt) = 0;

			bool logicComplete = false;
		};
	}
}