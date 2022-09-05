#pragma once
#include<functional>
#include<string>
namespace NCL {
	namespace CSC8503 {
		typedef std::function<void(float)>StateUpdateFunction;

		class State		
		{
		public:
			State(std::string name = "") : name(name) {}
			State(StateUpdateFunction someFunc, std::string name = "") : name(name) {
				func = someFunc;
			}
			virtual void Update(float dt) {
				if (func != nullptr) {
					func(dt);
				}
			}
			std::string GetName() {
				return name;
			}
		protected:
			StateUpdateFunction func;
			std::string name;

		};

		typedef void(*StateFunc)(void*);

		class GenericState : public State		{
		public:
			GenericState(StateFunc someFunc, void* someData) {
				func		= someFunc;
				funcData	= someData;
			}
			virtual void Update() {
				if (funcData != nullptr) {
					func(funcData);
				}
			}
		protected:
			StateFunc func;
			void* funcData;
		};
	}
}