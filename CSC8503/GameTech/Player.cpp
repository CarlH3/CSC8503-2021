#include "Player.h"

using namespace NCL::CSC8503;

void Player::OnCollisionBegin(GameObject* otherGameObject) {
	if (otherGameObject->GetName() == "coins" && otherGameObject->IsActive()) {
		otherGameObject->SetActive(false);
		(*score) += 25;
	}

	if (otherGameObject->GetName() == "endLine") {
		//otherGameObject->SetActive(false);
		reachedFinish = true;
	}
}

void Player::OnCollisionEnd(GameObject* otherGameObject) {
	if (otherGameObject->GetName() == "coins") {
		otherGameObject->CoinDeletion();
	}
}