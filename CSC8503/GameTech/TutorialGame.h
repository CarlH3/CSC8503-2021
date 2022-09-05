#pragma once
#include "GameTechRenderer.h"
#include "../CSC8503Common/PhysicsSystem.h"
#include "../CSC8503Common/StateGameObject.h"
#include "../CSC8503Common/PushdownMachine.h"
#include "../CSC8503Common/PushdownState.h"
#include"../CSC8503Common/GameObject.h"
#include"SideToSideGameObject.h"

#include"Player.h"

namespace NCL
{
	namespace CSC8503
	{
		class TutorialGame		
		{
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);
			//coursework function begin
			bool isSelected = false;
			void DrawMenu();
			void DrawWin();
			void DrawLose(std::string winner);
			void DrawPause();
			//void InitGameWorld1();//ball
			//void InitGameWorld2();//maze

			bool UpdatePushdown(float dt) {
				if (!machine->Update(dt)) {
					return false;
				}
				return true;
			}
			//coursework function end

		protected:
			//
			PushdownMachine* machine;
			std::string winnerName;
			Player* player = nullptr;//here
			bool multiplayer = 0;
			bool gameoneortwo = 0;
			//
			void InitialiseAssets();
			void InitCamera();

			void UpdateKeys();
			void StartNewGame();

			void InitWorld();
			void BuildWalls();
			void InitTestWorld();

			void InitGameExamples();

			void InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing);
			void InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);
			void InitDefaultFloor();
			void BridgeConstraintTest();
	
			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();

			StateGameObject* AddStateObjectToWorld(const Vector3& position);
			StateGameObject* testStateObject;


			GameObject* AddFloorToWorld(const Vector3& position, Vector3 size);
			GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f);
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			GameObject* AddOBBCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			
			GameObject* AddCapsuleToWorld(const Vector3& position, float halfHeight, float radius, float inverseMass = 10.0f);

			GameObject* AddPlayerToWorld(const Vector3& position);
			GameObject* AddEnemyToWorld(const Vector3& position);
			GameObject* AddBonusToWorld(const Vector3& position);

			void MakeObjectBounce(GameObject* object);
			void MakeObjectSmooth(GameObject* object);

			void SetColour(GameObject* object,Vector4 colour);


			GameTechRenderer*	renderer;
			PhysicsSystem*		physics;
			GameWorld*			world;

			bool				useGravity;
			bool				inSelectionMode;

			bool				debugMode;
			bool				returningToMenu;

			float				forceMagnitude;
			float				scoreTime;
			int					score;

			GameObject* selectionObject = nullptr;

			OGLMesh*	capsuleMesh = nullptr;
			OGLMesh*	cubeMesh	= nullptr;
			OGLMesh*	sphereMesh	= nullptr;
			OGLTexture* basicTex	= nullptr;
			OGLShader*	basicShader = nullptr;

			//Coursework Meshes
			OGLMesh*	charMeshA	= nullptr;
			OGLMesh*	charMeshB	= nullptr;
			OGLMesh*	enemyMesh	= nullptr;
			OGLMesh*	bonusMesh	= nullptr;

			//Coursework Additional functionality	
			GameObject* lockedObject	= nullptr;
			Vector3 lockedOffset		= Vector3(0, 14, 20);

			GameObject* rotating1 = nullptr;
			GameObject* rotating2 = nullptr;
			GameObject* rotating3 = nullptr;
			GameObject* rotating4 = nullptr;

			
			std::vector<GameObject*> apples;
			std::vector<AIGameObject*> aiObjects;

			void LockCameraToObject(GameObject* o) {
				lockedObject = o;
			}

		
		class IntroScreen :public PushdownState
		{//for intro menu screen 
		protected:
			TutorialGame* tugame;
			bool GameMode = 0;
		public:
			IntroScreen(TutorialGame* tugame) 
			{
				this->tugame = tugame;
			}
			PushdownResult OnUpdate(float dt, PushdownState** newstate) override 
			{
				
				if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM1)) 
				{
					*newstate = new GameScreen(tugame, 0);
					GameMode = 0;
					return PushdownResult::Push;
				}
				if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM2))
				{
					// game mode 2
					*newstate = new GameScreen(tugame, 1);
					GameMode = 1;
					return PushdownResult::Push;
				}
				if (Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE)) 
				{
					return PushdownResult::Pop;
				}
				tugame->DrawMenu();
				tugame->renderer->Update(dt);
				Debug::FlushRenderables(dt);
				tugame->renderer->RenderIntroMenu();

				
				return PushdownState::NoChange;
			}
			void OnAwake() override
			{
				std::cout << "Press 1 to play  or 2 for test world. ESC to quit\n";
			}

			void OnSleep() override 
			{
				if (GameMode == 0) 
				{//mode 1 game 
					tugame->StartNewGame();
					tugame->InitWorld();
				}
				if (GameMode == 1) 
				{//mode 2 test world
					tugame->StartNewGame();
					std::cout << "game mode 2 here " << std::endl;
					tugame->InitTestWorld();//just call gamemode2 function in here
				}
			}
		};
		
		class GameScreen :public PushdownState
		{
		protected:
			TutorialGame* tugame;
			float pausesave = 1;
			bool multiplayer;
		public:
			GameScreen(TutorialGame* tugame, bool multiplayer = 0) 
			{
				tugame->multiplayer = multiplayer;
				this->tugame = tugame;
			}
			PushdownResult OnUpdate(float dt, PushdownState** newstate) override
			{

				if (pausesave < 0) {
					if (Window::GetKeyboard()->KeyDown(KeyboardKeys::P))
					{
						
						return PushdownResult::Pop;
					}
					if (Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE))
					{
						*newstate = new PauseScreen(tugame);
						return PushdownResult::Push;
					}
				}
				else 
				{
					pausesave -= dt;
				}

				tugame->UpdateGame(dt);

				tugame->scoreTime += dt;

				if (tugame->scoreTime >= 1) {
					tugame->score -= 10;
					tugame->scoreTime = 0;
				}

				tugame->renderer->DrawString("Score:" + std::to_string(tugame->score), Vector2(80, 10), Vector4(0, 0, 0, 1));

				if (tugame->score <= 0) {
					tugame->score = 0;
					*newstate = new GameOverMenu(tugame);

					return PushdownResult::Push;
				}

				if (tugame->rotating1) {
					Transform& transform = tugame->rotating1->GetTransform();
					Quaternion orientation = transform.GetOrientation();

					orientation += (Quaternion(Vector3(0, dt * 0.25f, 0), 0.0f) * orientation);
					orientation.Normalise();

					transform.SetOrientation(orientation);
				}

				if (tugame->rotating2) {
					Transform& transform = tugame->rotating2->GetTransform();
					Quaternion orientation = transform.GetOrientation();

					orientation += (Quaternion(Vector3(0, dt * -0.25f, 0), 0.0f) * orientation);
					orientation.Normalise();

					transform.SetOrientation(orientation);
				}

				if (tugame->rotating3) {
					Transform& transform = tugame->rotating3->GetTransform();
					Quaternion orientation = transform.GetOrientation();

					orientation += (Quaternion(Vector3(0, dt * -0.25f, 0), 0.0f) * orientation);
					orientation.Normalise();

					transform.SetOrientation(orientation);
				}
				if (tugame->rotating4) {
					Transform& transform = tugame->rotating4->GetTransform();
					Quaternion orientation = transform.GetOrientation();

					orientation += (Quaternion(Vector3(0, dt * -0.25f, 0), 0.0f) * orientation);
					orientation.Normalise();

					transform.SetOrientation(orientation);
				}

				//
				if (tugame->player->reachedFinish) 
				{
					*newstate = new VictoryMenu(tugame);
					return PushdownResult::Push;
				}

				for (auto i : tugame->aiObjects) {
					i->UpdateAI(dt);
				}

	

			

				return PushdownResult::NoChange;
			}
			void OnAwake() override {
				std::cout << "Resuming Game\n";
				pausesave = 0.2;
			}
		};


		class PauseScreen :public PushdownState
		{//for player press pause buttom screen 
		protected:
			TutorialGame* tugame;
			float pausesave = 1;
		public:
			PauseScreen(TutorialGame* tugame)
			{
				this->tugame = tugame;
			}
			PushdownResult OnUpdate(float dt, PushdownState** newstate) override
			{
				if (pausesave < 0) {
					if (Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE))
					{
						return PushdownResult::Pop;
					}
				}
				else {
					pausesave -= dt;
				}

				tugame->renderer->DrawString("PAUSED", Vector2(30, 30), Vector4(0, 0, 0, 1), 70.0f);
				tugame->renderer->DrawString("Press ESC back to game", Vector2(24, 70), Vector4(0, 0, 0, 1));
				tugame->renderer->DrawString("Press ESC and then press P to quit to main menu", Vector2(10, 75), Vector4(0, 0, 0, 1));

				tugame->renderer->Update(dt);
				Debug::FlushRenderables(dt);
				tugame->renderer->Render();
				return PushdownResult::NoChange;
			}
			void OnAwake() override {
				std::cout << "press esc to pause game" << std::endl;
				pausesave = 0.2;
			}
		};

		class GameOverMenu : public PushdownState {
		public:
			GameOverMenu(TutorialGame* tugame) 
			{
				this->tugame = tugame;
			}


			PushdownResult OnUpdate(float dt, PushdownState** newstate) override
			{
				
					if (Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE))
					{
						*newstate = new IntroScreen(tugame);
						return PushdownResult::Push;
					}

					if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::RETURN)) {
						tugame->StartNewGame();
						tugame->InitWorld();
						*newstate = new GameScreen(tugame);

						return PushdownResult::Pop;
					}

				
				tugame->renderer->DrawString("GAME OVER", Vector2(20, 30), Vector4(0, 0, 0, 1), 70.0f);
				tugame->renderer->DrawString("Score:" + std::to_string(tugame->score), Vector2(43, 50), Vector4(0, 0, 0, 1));
				tugame->renderer->DrawString("Press ENTER to play again", Vector2(26, 70), Vector4(0, 0, 0, 1));
				tugame->renderer->DrawString("Press ESC to quit", Vector2(34, 75), Vector4(0, 0, 0, 1));

				tugame->renderer->Update(dt);
				Debug::FlushRenderables(dt);
				tugame->renderer->Render();

				return PushdownResult::NoChange;
			}

		protected:
			TutorialGame* tugame;
		};

		class VictoryMenu : public PushdownState 
		{
		public:
			VictoryMenu(TutorialGame* tugame)
			{
				this->tugame = tugame;
			}

			PushdownResult OnUpdate(float dt, PushdownState** newState) override
			{
				Window::GetWindow()->ShowOSPointer(false);
				Window::GetWindow()->LockMouseToWindow(true);

				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::ESCAPE)) {
					*newState = new IntroScreen(tugame);
					return PushdownResult::Pop;
				}

				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::RETURN)) {
					tugame->StartNewGame();
					tugame->InitWorld();
					*newState = new GameScreen(tugame);

					return PushdownResult::Pop;
				}

				tugame->renderer->DrawString("VICTORY", Vector2(26, 30), Vector4(0, 0, 0, 1), 70.0f);
				tugame->renderer->DrawString("Score:" + std::to_string(tugame->score), Vector2(43, 50), Vector4(0, 0, 0, 1));
				tugame->renderer->DrawString("Press ENTER to play again", Vector2(26, 70), Vector4(0, 0, 0, 1));
				tugame->renderer->DrawString("Press Enter to play again and then Press P to Menu", Vector2(5, 75), Vector4(0, 0, 0, 1));

				tugame->renderer->Update(dt);
				Debug::FlushRenderables(dt);
				tugame->renderer->Render();

				return PushdownResult::NoChange;
			}

		protected:
			TutorialGame* tugame;
		};

		};
	}
}

