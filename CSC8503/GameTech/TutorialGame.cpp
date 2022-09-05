#include "TutorialGame.h"
#include "../CSC8503Common/GameWorld.h"
#include "../../Plugins/OpenGLRendering/OGLMesh.h"
#include "../../Plugins/OpenGLRendering/OGLShader.h"
#include "../../Plugins/OpenGLRendering/OGLTexture.h"
#include "../../Common/TextureLoader.h"
#include "../CSC8503Common/PositionConstraint.h"
#include "../CSC8503Common/OrientationConstraint.h"
#include "../CSC8503Common/StateGameObject.h"
#include"SideToSideGameObject.h"
#include"Player.h"

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame()	{
	machine = new PushdownMachine(new IntroScreen(this));

	world		= new GameWorld();
	renderer	= new GameTechRenderer(*world);
	physics		= new PhysicsSystem(*world);
	returningToMenu = false;
	forceMagnitude	= 100.0f;
	useGravity		= false;
	inSelectionMode = false;

	score = 50;
	scoreTime = 0.0f;

	Debug::SetRenderer(renderer);

	InitialiseAssets();
}

void TutorialGame::StartNewGame() {
	apples.clear();
	//enemies.clear();
	aiObjects.clear();

	score = 1000;
	scoreTime = 0.0f;

	selectionObject = nullptr;
	lockedObject = nullptr;
	player = nullptr;

	world->ClearAndErase();
	physics->Clear();
}
/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	auto loadFunc = [](const string& name, OGLMesh** into) {
		*into = new OGLMesh(name);
		(*into)->SetPrimitiveType(GeometryPrimitive::Triangles);
		(*into)->UploadToGPU();
	};

	loadFunc("cube.msh"		 , &cubeMesh);
	loadFunc("sphere.msh"	 , &sphereMesh);
	loadFunc("Male1.msh"	 , &charMeshA);
	loadFunc("courier.msh"	 , &charMeshB);
	loadFunc("security.msh"	 , &enemyMesh);
	loadFunc("coin.msh"		 , &bonusMesh);
	loadFunc("capsule.msh"	 , &capsuleMesh);

	basicTex	= (OGLTexture*)TextureLoader::LoadAPITexture("checkerboard.png");
	basicShader = new OGLShader("GameTechVert.glsl", "GameTechFrag.glsl");

	InitCamera();
	//InitWorld();
}

TutorialGame::~TutorialGame()	{
	delete cubeMesh;
	delete sphereMesh;
	delete charMeshA;
	delete charMeshB;
	delete enemyMesh;
	delete bonusMesh;

	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
	delete lockedObject;

	delete rotating1;
	delete rotating2;
	delete rotating3;
	delete rotating4;
}

void TutorialGame::UpdateGame(float dt) {
	
	UpdateKeys();

	if (!inSelectionMode) {
		world->GetMainCamera()->UpdateCamera(dt);
	}

	if (useGravity) {
		Debug::Print("(G)ravity on", Vector2(5, 95));
	}
	else {
		Debug::Print("(G)ravity off", Vector2(5, 95));
	}

	SelectObject();
	MoveSelectedObject();
	physics->Update(dt);

	
	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetPosition();

		Vector3 camPos;

		camPos = lockedObject->GetTransform().GetOrientation() * lockedOffset + objPos;


		Matrix4 temp = Matrix4::BuildViewMatrix(camPos, objPos, Vector3(0, 1, 0));

		Matrix4 modelMat = temp.Inverse();

		Quaternion q(modelMat);
		Vector3 angles = q.ToEuler(); //nearly there now!

		world->GetMainCamera()->SetPosition(camPos);
		world->GetMainCamera()->SetPitch(angles.x + 5);
		world->GetMainCamera()->SetYaw(angles.y);

		//Debug::DrawAxisLines(lockedObject->GetTransform().GetMatrix(), 2.0f);
	}

	




	world->UpdateWorld(dt);
	renderer->Update(dt);

	Debug::FlushRenderables(dt);
	renderer->Render();
}

void TutorialGame::UpdateKeys() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
		lockedObject	= nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::G)) {
		useGravity = !useGravity; //Toggle gravity!
		physics->UseGravity(useGravity);
	}
	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F8)) {
		world->ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement();
	}
	else {
		DebugObjectMovement();
	}
}

void TutorialGame::LockedObjectMovement() {
	Matrix4 view = world->GetMainCamera()->BuildViewMatrix();
	Matrix4 camWorld = view.Inverse();

	Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); //view is inverse of model!

	//forward is more tricky -  camera forward is 'into' the screen...
	//so we can take a guess, and use the cross of straight up, and
	//the right axis, to hopefully get a vector that's good enough!

	Vector3 charForward = lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, -1);

	float force = 150.0f;

	Transform& transform = lockedObject->GetTransform();
	Quaternion orientation = transform.GetOrientation();

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::A)) {
		lockedObject->GetPhysicsObject()->AddTorque(Vector3(0, 20, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::D)) {
		lockedObject->GetPhysicsObject()->AddTorque(Vector3(0, -20, 0));
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::W)) {
		lockedObject->GetPhysicsObject()->AddForce(charForward * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::S)) {
		lockedObject->GetPhysicsObject()->AddForce(-charForward * force);
	}
}

void TutorialGame::DebugObjectMovement() {
//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
	}

}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(0.1f);
	world->GetMainCamera()->SetFarPlane(1500.0f);
	world->GetMainCamera()->SetPitch(-15.0f);
	world->GetMainCamera()->SetYaw(315.0f);
	world->GetMainCamera()->SetPosition(Vector3(0, 10, 0));
	lockedObject = nullptr;
}

void TutorialGame::InitWorld() {
	debugMode = false;
	useGravity = true;
	physics->UseGravity(true);

	world->ClearAndErase();
	physics->Clear();
	InitCamera();

	InitGameExamples();
	BuildWalls();
	InitDefaultFloor();


	//rotated boxes
	rotating1 =	AddOBBCubeToWorld(Vector3(50, 20, 120), Vector3(5, 25, 5), 0.0f);
	rotating2 = AddOBBCubeToWorld(Vector3(-80, 10, 120), Vector3(5, 10, 5), 0.0f);
	rotating3 = AddOBBCubeToWorld(Vector3(110, 20, 65), Vector3(5, 25, 5), 0.0f);
	rotating4 = AddOBBCubeToWorld(Vector3(90, 40, 95), Vector3(10, 20, 5), 0.0f);


	// coins first part
	AddBonusToWorld(Vector3(30, 3, -50));
	AddBonusToWorld(Vector3(50, 3, -50));
	AddBonusToWorld(Vector3(70, 3, -50));
	AddBonusToWorld(Vector3(30, 3, -70));
	AddBonusToWorld(Vector3(50, 3, -70));
	AddBonusToWorld(Vector3(70, 3, -70));
	AddBonusToWorld(Vector3(30, 3, -90));
	AddBonusToWorld(Vector3(50, 3, -90));
	AddBonusToWorld(Vector3(70, 3, -90));
	//coins second part
	AddBonusToWorld(Vector3(30, 3, 70));
	AddBonusToWorld(Vector3(50, 3, 70));
	AddBonusToWorld(Vector3(70, 3, 70));
	AddBonusToWorld(Vector3(30, 3, 90));
	AddBonusToWorld(Vector3(50, 3, 90));
	AddBonusToWorld(Vector3(70, 3, 90));
	AddBonusToWorld(Vector3(30, 3, 110));
	AddBonusToWorld(Vector3(50, 3, 110));
	AddBonusToWorld(Vector3(70, 3, 110));
	// coins third part
	AddBonusToWorld(Vector3(10, 3, 170));
	AddBonusToWorld(Vector3(-10, 3, 170));
	AddBonusToWorld(Vector3(30, 3, 170));
	AddBonusToWorld(Vector3(-30, 3, 170));
	AddBonusToWorld(Vector3(30, 3, 150));
	AddBonusToWorld(Vector3(50, 3, 150));
	AddBonusToWorld(Vector3(70, 3, 150));
	AddBonusToWorld(Vector3(90, 3, 150));

	Vector3 dimensions = Vector3(50, 2, 10);

	SideToSideGameObject* MovingCube = new SideToSideGameObject(1000, "MovingCube");
	dimensions = Vector3(5, 5, 5);
	OBBVolume* volume = new OBBVolume(dimensions);
	

	MovingCube->SetBoundingVolume((CollisionVolume*)volume);
	MovingCube->GetTransform().SetPosition(Vector3(50, 15, 50)).SetScale(dimensions * 2);
	MovingCube->SetRenderObject(new RenderObject(&MovingCube->GetTransform(), cubeMesh, basicTex, basicShader));
	MovingCube->SetPhysicsObject(new PhysicsObject(&MovingCube->GetTransform(), MovingCube->GetBoundingVolume()));
	MovingCube->GetPhysicsObject()->SetInverseMass(0.1f);
	MovingCube->GetPhysicsObject()->InitCubeInertia();


	world->AddGameObject(MovingCube);
	aiObjects.emplace_back(MovingCube);
	

	SideToSideGameObject* MovingMan = new SideToSideGameObject(1000, "MovingMan");
	float hf = 4.0f;
	float r = 3.0f;
	CapsuleVolume* volume1 = new CapsuleVolume(hf,r);

	MovingMan->SetBoundingVolume((CollisionVolume*)volume1);
	MovingMan->GetTransform().SetPosition(Vector3(60, 8, 90)).SetScale(dimensions * 2);
	MovingMan->SetRenderObject(new RenderObject(&MovingMan->GetTransform(), charMeshA, basicTex, basicShader));
	MovingMan->SetPhysicsObject(new PhysicsObject(&MovingMan->GetTransform(), MovingMan->GetBoundingVolume()));
	MovingMan->GetPhysicsObject()->SetInverseMass(0.1f);
	MovingMan->GetPhysicsObject()->InitCubeInertia();
	world->AddGameObject(MovingMan);
	aiObjects.emplace_back(MovingMan);

	// THE END floor
	GameObject* endLine = AddCubeToWorld(Vector3(50, 18, 50), Vector3(50, 3, 50), 0.0f);
	endLine->SetName("endLine");
	//the mark of end line
	GameObject* finishLineFrame = AddCubeToWorld(Vector3(50, 22.5f, 50), Vector3(50, 2.5f, 2.5f), 0.0f);
	finishLineFrame->SetDefaultColour(Debug::BLACK);
	finishLineFrame->GetRenderObject()->SetColour(finishLineFrame->GetDefaultColour());


}

void TutorialGame::InitTestWorld()
{
	debugMode = true;
	useGravity = false;
	physics->UseGravity(false);
	

	AddPlayerToWorld(Vector3(-50, 5, 0));

	InitCamera();
	AddFloorToWorld(Vector3(0, -2, 0), Vector3(100, 2, 100));
	MakeObjectBounce(AddFloorToWorld(Vector3(200, -2, 0), Vector3(100, 2, 100)));
	MakeObjectSmooth(AddFloorToWorld(Vector3(0, -2, -200), Vector3(100, 2, 100)));

	AddCubeToWorld(Vector3(-20, 10, 23.5), Vector3(1.0f, 1.0f, 1.0f));
	AddCubeToWorld(Vector3(-23, 10, 23.5), Vector3(1.0f, 1.0f, 1.0f));
	AddOBBCubeToWorld(Vector3(-20, 10, 23), Vector3(1.0f, 1.0f, 1.0f));
	AddOBBCubeToWorld(Vector3(-23.5, 10, 20), Vector3(1.0f, 1.0f, 1.0f));
	AddSphereToWorld(Vector3(-27, 10, 20), 1.0f);
	AddSphereToWorld(Vector3(-30.5, 10, 20), 1.0f);
	AddCapsuleToWorld(Vector3(-27, 10, 23.5), 1.0f, 1.0f);
	AddCapsuleToWorld(Vector3(-30.5, 10, 23.5), 1.0f, 1.0f);

	/*
	GameObject* cube = AddCubeToWorld(Vector3(0, 50, 0), Vector3(1.0f, 1.0f, 1.0f));
	PositionConstraint* constraint = new PositionConstraint(cube, Vector3(0, 70, 0), 20.0f);
	world->AddConstraint(constraint);
	OrientationConstraint* constraint2 = new OrientationConstraint(cube, Vector3(0, 70, -0), 0.0f);
	world->AddConstraint(constraint2);
	*/

}
/*
void TutorialGame::BridgeConstraintTest() {
	Vector3 cubeSize = Vector3(8, 8, 8);

	float invCubeMass = 5; //how heavy the middle pieces are
	int numLinks = 10;
	float maxDistance = 30; // constraint distance
	float cubeDistance = 20; // distance between links

	Vector3 startPos = Vector3(500, 500, 500);

	GameObject* start = AddCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize, 0);
	GameObject* end = AddCubeToWorld(startPos + Vector3((numLinks + 2) * cubeDistance, 0, 0), cubeSize, 0);

	GameObject* previous = start;

	for (int i = 0; i < numLinks; ++i) {
		GameObject* block = AddCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, 0, 0), cubeSize, invCubeMass);

		PositionConstraint* constraint = new PositionConstraint(previous, block, maxDistance);
		world->AddConstraint(constraint);
		previous = block;
	}
	PositionConstraint* constraint = new PositionConstraint(previous, end, maxDistance);
	world->AddConstraint(constraint);
}
*/
/*

A single function to add a large immoveable cube to the bottom of our world

*/
GameObject* TutorialGame::AddFloorToWorld(const Vector3& position, Vector3 size) {
	GameObject* floor = AddCubeToWorld(position, size, 0);
	floor->SetName("floor");

	return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple' 
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddCapsuleToWorld(const Vector3& position, float halfHeight, float radius, float inverseMass) {
	GameObject* capsule = new GameObject();

	CapsuleVolume* volume = new CapsuleVolume(halfHeight, radius);
	capsule->SetBoundingVolume((CollisionVolume*)volume);

	capsule->GetTransform()
		.SetScale(Vector3(radius* 2, halfHeight, radius * 2))
		.SetPosition(position);

	capsule->SetRenderObject(new RenderObject(&capsule->GetTransform(), capsuleMesh, basicTex, basicShader));
	capsule->SetPhysicsObject(new PhysicsObject(&capsule->GetTransform(), capsule->GetBoundingVolume()));

	capsule->GetPhysicsObject()->SetInverseMass(inverseMass);
	capsule->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(capsule);

	return capsule;

}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddOBBCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

void TutorialGame::InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f);
		}
	}
	AddFloorToWorld(Vector3(0, -2, 0), Vector3(100, 2, 100));
}

void TutorialGame::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 1.0f;
	Vector3 cubeDims = Vector3(1, 1, 1);

	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

			if (rand() % 2) {
				AddCubeToWorld(position, cubeDims);
			}
			else {
				AddSphereToWorld(position, sphereRadius);
			}
		}
	}
}

void TutorialGame::InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	for (int x = 1; x < numCols+1; ++x) {
		for (int z = 1; z < numRows+1; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f);
		}
	}
}

void TutorialGame::InitDefaultFloor() {
	AddFloorToWorld(Vector3(-50, -2, -45), Vector3(50, 2, 50));
	AddFloorToWorld(Vector3(-50, -2, 55), Vector3(50, 2, 50));
	AddFloorToWorld(Vector3(-50, -2, 155), Vector3(50, 2, 50));

	
	AddFloorToWorld(Vector3(50, -2, -45), Vector3(50, 2, 50));
	MakeObjectSmooth(AddFloorToWorld(Vector3(50, -2, 55), Vector3(50, 2, 50)));
	AddFloorToWorld(Vector3(50, -2, 155), Vector3(50, 2, 50));

	AddFloorToWorld(Vector3(150, -2, -45), Vector3(50, 2, 50));
	AddFloorToWorld(Vector3(150, -2, 55), Vector3(50, 2, 50));
	MakeObjectBounce(AddFloorToWorld(Vector3(150, -2, 155), Vector3(50, 2, 50)));

	
	
}

void TutorialGame::InitGameExamples() {

	lockedObject = AddPlayerToWorld(Vector3(-72, 10, 162));


	SetColour(AddCubeToWorld(Vector3(-20, 10, 23.5), Vector3(10.0f, 10.0f, 10.0f),0), Vector4(0.3f, 0.1f, 0.05f, 1));
	SetColour(AddCubeToWorld(Vector3(123, 10, 23.5), Vector3(10.0f, 10.0f, 10.0f),0), Vector4(0.4f, 0.3f, 0.9f, 1));


	SetColour(AddSphereToWorld(Vector3(50, 10, 155), 10.0f,0), Vector4(0.1f, 0.1f, 0.9f, 1));
	SetColour(AddSphereToWorld(Vector3(-50, 10, 155), 10.0f,0), Vector4(0.8f, 0.7f, 0.05f, 1));

	SetColour(AddCapsuleToWorld(Vector3(-57, 10, 135.5), 15.0f, 10.0f,0), Vector4(0.1f, 0.2f, 0.1f, 1));
	SetColour(AddCapsuleToWorld(Vector3(50.5, 10, -50.5), 15.0f, 10.0f,0), Vector4(0.5f, 0.75f, 1, 1));

	SetColour(AddCubeToWorld(Vector3(50, 18, 45), Vector3(3.0f, 3.0f, 3.0f)), Vector4(0.25f, 0.5f, 0.6f, 1));
	SetColour(AddCubeToWorld(Vector3(35, 18, 70), Vector3(4.0f, 4.0f, 4.0f)), Vector4(0.25f, 0.5f, 1, 1));
			 
	SetColour(AddOBBCubeToWorld(Vector3(30, 18, 23), Vector3(6.0f, 6.0f, 6.0f)), Vector4(0.25f, 0.5f, 1, 1));
	SetColour(AddOBBCubeToWorld(Vector3(23.5, 18, 20), Vector3(3.0f, 3.0f, 3.0f)), Vector4(0.25f, 0.5f, 1, 1));
			 
	SetColour(AddSphereToWorld(Vector3(150, 7, -40), 6.0f), Vector4(0.25f, 0.5f, 1, 1));
	SetColour(AddSphereToWorld(Vector3(100, 8, 20), 6.0f), Vector4(0.25f, 0.5f, 1, 1));
			 
	SetColour(AddCapsuleToWorld(Vector3(-22, 25, 27), 3.0f, 2.0f), Vector4(0.25f, 0.5f, 1, 1));
	SetColour(AddCapsuleToWorld(Vector3(22, 15, 23.5), 3.0f, 2.0f), Vector4(0.25f, 0.5f, 1, 1));

}

void TutorialGame::BuildWalls()
{
	//Build walls
	
	MakeObjectBounce(AddCubeToWorld(Vector3(-25, 10, 200), Vector3(75, 10, 5), 0.0f));
	MakeObjectBounce(AddCubeToWorld(Vector3(125, 10, 200), Vector3(75, 10, 5), 0.0f));

	MakeObjectBounce(AddCubeToWorld(Vector3(-25, 8, -100), Vector3(75, 10, 5), 0.0f));
	MakeObjectBounce(AddCubeToWorld(Vector3(125, 8, -100), Vector3(75, 10, 5), 0.0f));

	MakeObjectBounce(AddCubeToWorld(Vector3(200, 8, 100), Vector3(5, 10, 100), 0.0f));
	MakeObjectBounce(AddCubeToWorld(Vector3(200, 8, 0), Vector3(5, 10, 100), 0.0f));

	MakeObjectBounce(AddCubeToWorld(Vector3(-100, 8, 100), Vector3(5, 10, 100), 0.0f));
	MakeObjectBounce(AddCubeToWorld(Vector3(-100, 8, 0), Vector3(5, 10, 100), 0.0f));
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	float meshSize = 3.0f;
	float inverseMass = 1.0f;

	

	player = new Player("player", &score);

	SphereVolume* volume = new SphereVolume(0.55f * meshSize);

	player->SetBoundingVolume((CollisionVolume*)volume);

	player->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	if (rand() % 2) {
		player->SetRenderObject(new RenderObject(&player->GetTransform(), sphereMesh, nullptr, basicShader));
	}
	else {
		player->SetRenderObject(new RenderObject(&player->GetTransform(), sphereMesh, nullptr, basicShader));
	}

	player->GetRenderObject()->SetColour(player->GetDefaultColour());

	player->SetPhysicsObject(new PhysicsObject(&player->GetTransform(), player->GetBoundingVolume()));
	player->GetPhysicsObject()->SetInverseMass(inverseMass);
	player->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(player);

	return player;
}

GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize		= 3.0f;
	float inverseMass	= 0.5f;

	GameObject* character = new GameObject();

	AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	return character;
}

GameObject* TutorialGame::AddBonusToWorld(const Vector3& position) {
	GameObject* apple = new GameObject("coins");

	SphereVolume* volume = new SphereVolume(2.25f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetDefaultColour(Debug::YELLOW);
	apple->GetRenderObject()->SetColour(apple->GetDefaultColour());

	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));
	apple->GetPhysicsObject()->SetInverseMass(10.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);
	apples.emplace_back(apple);

	return apple;

}

StateGameObject* TutorialGame::AddStateObjectToWorld(const Vector3& position) {
	StateGameObject* apple = new StateGameObject();

	SphereVolume* volume = new SphereVolume(0.25f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}

/*

Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be 
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around. 

*/
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	if (inSelectionMode) {
		Window::GetWindow()->ShowOSPointer(true);
		Window::GetWindow()->LockMouseToWindow(false);
		renderer->DrawString("Press Q to change to camera mode!", Vector2(5, 85));

		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::LEFT)) {
			if (selectionObject) {	//set colour to deselected;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				selectionObject = nullptr;
				lockedObject	= nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {
				selectionObject = (GameObject*)closestCollision.node;
				selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				return true;
			}
			else {
				return false;
			}
		}
	}
	else {
		renderer->DrawString("Press Q to change to select mode!", Vector2(5, 85));
	}

	if (lockedObject) {
		renderer->DrawString("Press L to unlock object!", Vector2(5, 80));
	}

	else if(selectionObject){
		int xPos = round(selectionObject->GetTransform().GetPosition().x);
		int yPos = round(selectionObject->GetTransform().GetPosition().y);
		int zPos = round(selectionObject->GetTransform().GetPosition().z);

		std::string s = "(" + std::to_string(xPos) + ", " + std::to_string(yPos) + ", " + std::to_string(zPos) + ")";
		renderer->DrawString("Position: " + s, Vector2(75, 50), Vector4(0.75f, 0.75f, 0.75f, 1.0f), 12.0f);


		float x = selectionObject->GetTransform().GetOrientation().x;
		float y = selectionObject->GetTransform().GetOrientation().y;
		float z = selectionObject->GetTransform().GetOrientation().z;
		float w = selectionObject->GetTransform().GetOrientation().w;

		s = "(" + std::to_string(x) + ", " + std::to_string(y) + ", ";
		renderer->DrawString("Orientation: " + s, Vector2(61, 55), Vector4(0.75f, 0.75f, 0.75f, 1.0f), 12.0f);
		s = std::to_string(z) + ", " + std::to_string(w) + ")";
		renderer->DrawString(s, Vector2(77, 58), Vector4(0.75f, 0.75f, 0.75f, 1.0f), 12.0f);

		renderer->DrawString("Press L to lock selected object object!", Vector2(5, 80));
	}

	if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::L)) {
		if (selectionObject) {
			if (lockedObject == selectionObject) {
				lockedObject = nullptr;
			}
			else {
				lockedObject = selectionObject;
			}
		}

	}


	return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/
void TutorialGame::MoveSelectedObject() {
	//Draw debug text at 10, 20
	renderer->DrawString("Click Force:" + std::to_string(forceMagnitude),Vector2(10, 20));

	forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;

	if (!selectionObject)
		return;//we haven¡¯t selected anything!

	//Push the selected object!
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::RIGHT)) {
		Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

		RayCollision closestCollision;

		if (world->Raycast(ray, closestCollision, true)) {
			if (closestCollision.node == selectionObject)
				selectionObject->GetPhysicsObject()->
				AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
		}
	}
}

void TutorialGame::MakeObjectBounce(GameObject* object) 
{
	object->GetPhysicsObject()->SetElasticity(4.0f);
	Vector4 colour = Debug::MAGENTA;
	object->SetDefaultColour(colour);
	object->GetRenderObject()->SetColour(colour);
}

void TutorialGame::MakeObjectSmooth(GameObject* object) 
{
	object->GetPhysicsObject()->SetFriction(0.001f);
	Vector4 colour = Vector4(0.5f, 0.65f, 1, 1);
	object->SetDefaultColour(colour);
	object->GetRenderObject()->SetColour(colour);
}
void TutorialGame::SetColour(GameObject* object, Vector4 colour)
{
	object->SetDefaultColour(colour);
	object->GetRenderObject()->SetColour(colour);
}

//coursework function begin
void TutorialGame::DrawMenu() {
	//glClearColor(0, 0, 0, 1);
	//Debug::FlushRenderables(0);
	renderer->DrawString("Welcome to Demo", Vector2(10, 10));
	renderer->DrawString("CSC8503 ZhengYang He", Vector2(10, 20));
	renderer->DrawString("200245470", Vector2(10, 30));
	renderer->DrawString("Press '1' for game", Vector2(10, 40));
	renderer->DrawString("Press '2' for test World", Vector2(10, 50));
	renderer->DrawString("Press 'ESC' quit", Vector2(10, 60));
	//renderer->Render();
	//world->ClearAndErase();
	//player = nullptr;
	//physics->Clear();
	//winnerName.clear();
}
void TutorialGame::DrawWin() {
	renderer->DrawString("you win", Vector2(10, 10));
}
void TutorialGame::DrawLose(std::string winnner) {
	renderer->DrawString("you lose", Vector2(10, 10));
}
void TutorialGame::DrawPause() {
	renderer->DrawString("now game paused", Vector2(10, 50));
}

//coursework function end