#include "../../Extras/Serialize/BulletFileLoader/bFile.h"
#include <btBulletDynamicsCommon.h>
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../../Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h"
#include "../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "../../Extras/Serialize/BulletWorldImporter/btWorldImporter.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btVector3.h"

#include <gtest/gtest.h>

static btMultiBodyDynamicsWorld *create_btMultiBodyDynamicsWorld()
{
	btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();

	btCollisionDispatcher * dispatcher = new btCollisionDispatcher(collisionConfiguration);

	btDbvtBroadphase * broadphase = new btDbvtBroadphase();

	btMultiBodyConstraintSolver* solver = new btMultiBodyConstraintSolver();

	btMultiBodyDynamicsWorld * world = new btMultiBodyDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	world->setGravity(btVector3(0, -10, 0));

	return world;

}

btRigidBody* CreateBox(btVector3 position, btVector3 velocity, btVector3 inertia, btScalar mass = 1.f)
{
	btBoxShape* colShape = new btBoxShape(btVector3(.1, .1, .1));
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(position);
	colShape->calculateLocalInertia(mass, inertia);
	btRigidBody* box = new btRigidBody(mass, 0, colShape, inertia);
	box->setWorldTransform(startTransform);
	box->setUserIndex(-1);
	box->setLinearVelocity(velocity);
	return box;
}

GTEST_TEST(BulletDynamics, DeterministicSaveRestore)
{

	static char filename[] = "test_serialize.bullet";
	btRigidBody* box_before[3];
	btCollisionObject* box_after[3];
	btMultiBodyDynamicsWorld *initial_world = create_btMultiBodyDynamicsWorld();

	// create ground plane and box shapes
	// in test three boxes are created

	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	{
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
		btScalar mass(0.);
		btVector3 localInertia(0, 0, 0);
		groundShape->calculateLocalInertia(mass, localInertia);
		btRigidBody* body = new btRigidBody(mass, 0, groundShape, localInertia);
		body->setWorldTransform(groundTransform);
		body->setUserIndex(-1);
		initial_world->addRigidBody(body);
	}
	{
		btVector3 localInertia(0, 0, 0);

	//set first position of boxes and linear velocity

		box_before[0] = CreateBox(btVector3(2,10,0), btVector3(-1,0,1), localInertia);
		initial_world->addRigidBody(box_before[0]);
		box_before[1] = CreateBox(btVector3(2,10,2), btVector3(-1,0,-1), localInertia);
		initial_world->addRigidBody(box_before[1]);
		box_before[2] = CreateBox(btVector3(0,10,1), btVector3(1,0,0), localInertia);
		initial_world->addRigidBody(box_before[2]);
	}

	// step simulation until collision

	const double delta_t = 0.01;
	int time = 0;
	while (box_before[0]->getCenterOfMassPosition().getY() == box_before[1]->getCenterOfMassPosition().getY() &&
			box_before[1]->getCenterOfMassPosition().getY() == box_before[2]->getCenterOfMassPosition().getY())
	{
		time++;
		initial_world->stepSimulation(delta_t);
	}

	// serialize

	if (initial_world->getDispatcher()->getNumManifolds() < 2)
	{
		printf("Error: too little manifolds\n");
		exit(1);
	}

	const btVector3 before_serialize1 = box_before[0]->getCenterOfMassPosition();
	const btVector3 before_serialize2 = box_before[1]->getCenterOfMassPosition();
	const btVector3 before_serialize3 = box_before[2]->getCenterOfMassPosition();
	btSerializer* s = new btDefaultSerializer;
	initial_world->serialize(s);
	FILE* file = fopen(filename, "wb");
	fwrite(s->getBufferPointer(), s->getCurrentBufferSize(), 1, file);
	fclose(file);

	// step simulation until boxes are no longer moving

	for (int i = 0; i < 500; ++i)
	{
		time++;
		initial_world->stepSimulation(delta_t);
	}

	// save position of all boxes

	const btVector3 steady_state1 = box_before[0]->getCenterOfMassPosition();
	const btVector3 steady_state2 = box_before[1]->getCenterOfMassPosition();
	const btVector3 steady_state3 = box_before[2]->getCenterOfMassPosition();

	// deserialize

	btMultiBodyDynamicsWorld *deserialized_world = create_btMultiBodyDynamicsWorld();
	btMultiBodyWorldImporter importer(deserialized_world);
	importer.loadFile(filename);
	box_after[0] = importer.getRigidBodyByIndex(1);
	box_after[1] = importer.getRigidBodyByIndex(2);
	box_after[2] = importer.getRigidBodyByIndex(3);

	// step simulation until boxes are no longer moving

	for (int i = 0; i < 500; ++i)
	{
		time++;
		deserialized_world->stepSimulation(delta_t);
	}

	const btVector3 after_deserialize1 = box_after[0]->getCenterOfMassPosition();
	const btVector3 after_deserialize2 = box_after[1]->getCenterOfMassPosition();
	const btVector3 after_deserialize3 = box_after[2]->getCenterOfMassPosition();

	// assert that the position of all boxes in deserialized_world are the same as in initial_world
/*
	EXPECT_FLOAT_EQ(steady_state1.getX(),after_deserialize1.getX());
//	EXPECT_FLOAT_EQ(steady_state1.getY(),after_deserialize1.getY());
	EXPECT_FLOAT_EQ(steady_state1.getZ(),after_deserialize1.getZ());
	EXPECT_FLOAT_EQ(steady_state2.getX(),after_deserialize2.getX());
	EXPECT_FLOAT_EQ(steady_state2.getY(),after_deserialize2.getY());
	EXPECT_FLOAT_EQ(steady_state2.getZ(),after_deserialize2.getZ());
	EXPECT_FLOAT_EQ(steady_state3.getX(),after_deserialize3.getX());
	EXPECT_FLOAT_EQ(steady_state3.getY(),after_deserialize3.getY());
	EXPECT_FLOAT_EQ(steady_state3.getZ(),after_deserialize3.getZ());
*/
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

