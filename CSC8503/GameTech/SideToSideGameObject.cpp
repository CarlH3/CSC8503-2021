#include "SideToSideGameObject.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/State.h"

using namespace NCL;
using namespace CSC8503;

SideToSideGameObject::SideToSideGameObject(float forceMagnitude, std::string name) : StateGameObject(name), forceMagnitude(forceMagnitude) {
	State* stateA = new State(
		[&](float dt)->void {
			this->MoveLeft(dt);
		}, std::string("MovingLeft")
			);

	State* stateB = new State(
		[&](float dt)->void {
			this->MoveRight(dt);
		}, std::string("MovingRight")
			);

	stateMachine->AddState(stateA);
	stateMachine->AddState(stateB);

	stateMachine->AddTransition(new StateTransition(stateA, stateB,
		[&]()->bool {
			return this->GetTransform().GetPosition().x < 25.0f;
		}
	));

	stateMachine->AddTransition(new StateTransition(stateB, stateA,
		[&]()->bool {
			return this->GetTransform().GetPosition().x > 75.0f;
		}
	));
}

void SideToSideGameObject::MoveLeft(float dt) {
	GetPhysicsObject()->AddForce(Vector3(-1, 0, 0) * forceMagnitude);
}

void SideToSideGameObject::MoveRight(float dt) {
	GetPhysicsObject()->AddForce(Vector3(1, 0, 0) * forceMagnitude);
}