#pragma once
#include "../CSC8503Common/GameObject.h"

namespace NCL {
	namespace CSC8503 {
		class Player : public GameObject {
		public:
			Player(string name, int* score) : GameObject(name), score(score), reachedFinish(false) 
			{ 
				defaultColour = Vector4(0, 0, 1, 1);
			}

			void OnCollisionBegin(GameObject* otherObject) override;
			void OnCollisionEnd(GameObject* otherObject) override;

			
			bool reachedFinish;

		protected:
			int* score;
		};
	}
}

