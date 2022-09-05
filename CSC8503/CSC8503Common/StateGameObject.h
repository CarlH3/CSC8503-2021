#pragma once
#include "../CSC8503Common/AIGameObject.h"
#include "../CSC8503Common/StateMachine.h"

namespace NCL {
	namespace CSC8503 {
		class StateGameObject : public AIGameObject {
		public:
			StateGameObject(std::string name = "") : AIGameObject(name), stateMachine(new StateMachine()) {};
			~StateGameObject() { delete stateMachine; }

			void UpdateAI(float dt) override {
				stateMachine->Update(dt);
			}

			State* GetActiveState() {
				return stateMachine->GetActiveState();
			}

		protected:
			StateMachine* stateMachine;
		};
	}
}