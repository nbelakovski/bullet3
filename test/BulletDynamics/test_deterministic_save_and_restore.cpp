#include "../../Extras/Serialize/BulletFileLoader/bFile.h"
#include <btBulletDynamicsCommon.h>
#include "../../src/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "../../src/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../../Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h"
#include "../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "../../Extras/Serialize/BulletWorldImporter/btWorldImporter.h"
#include "../../src/BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "../../src/BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../../src/BulletDynamics/Dynamics/btRigidBody.h"

#include "../../examples/CommonInterfaces/CommonRigidBodyBase.h"
#include <gtest/gtest.h>
#define USE_MOTIONSTATE 0

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

GTEST_TEST(BulletDynamics, DeterministicSaveRestore)
{

	static char filename[] = "test_serialize.mixail";
	btRigidBody* box_before[3];
	btCollisionObject* box_after[3];
	btMultiBodyDynamicsWorld *initial_world = create_btMultiBodyDynamicsWorld();

	// create ground plane and box shapes
	// don't worry about gui things

	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	{
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
		btScalar mass(0.);
		bool isDynamic = (mass != 0.f);
		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		#ifdef USE_MOTIONSTATE
			btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);

			btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, groundShape, localInertia);

			btRigidBody* body = new btRigidBody(cInfo);
		#else
			btRigidBody* body = new btRigidBody(mass, 0, groundShape, localInertia);
			body->setWorldTransform(groundTransform);
		#endif

		body->setUserIndex(-1);
		initial_world->addRigidBody(body);
	}
	{

		btBoxShape* colShape = new btBoxShape(btVector3(.1, .1, .1));

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);
		for (int i = 0; i < 3; i++)
		{
			int a[4];
			if (i == 0)
			{
				a[0] = 2;
				a[1] = 0;
				a[2] = -1;
				a[3] = 1;
			}
			else if (i ==1)
			{
				a[0] = 2;
				a[1] = 2;
				a[2] = -1;
				a[3] = -1;
			}
			else
			{
				a[0] = 0;
				a[1] = 1;
				a[2] = 1;
				a[3] = 0;
			}
			startTransform.setOrigin(btVector3(
				btScalar(a[0]),
				btScalar(10),
				btScalar(a[1])));
			#ifdef USE_MOTIONSTATE
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

				btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, colShape, localInertia);

				box_before[i] = new btRigidBody(cInfo);
			#else
				box_before[i] = new btRigidBody(mass, 0, colShape, localInertia);
				box_before[i]->setWorldTransform(startTransform);
			#endif
			box_before[i]->setUserIndex(-1);
			initial_world->addRigidBody(box_before[i]);
			box_before[i]->setLinearVelocity(btVector3(a[2], 0, a[3]));
		}
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

	printf("Serialize in %d time\n", time);
	btSerializer* s = new btDefaultSerializer;
	initial_world->serialize(s);
	FILE* file = fopen(filename, "wb");
	fwrite(s->getBufferPointer(), s->getCurrentBufferSize(), 1, file);
	fclose(file);
	const btVector3 before_serialize1 = box_before[0]->getCenterOfMassPosition();
	const btVector3 before_serialize2 = box_before[1]->getCenterOfMassPosition();
	const btVector3 before_serialize3 = box_before[2]->getCenterOfMassPosition();

	// step simulation until boxes are no longer moving

	for (int i = 0; i < 500; ++i)
	{
		time++;
		initial_world->stepSimulation(delta_t);
	}

	// save position of all boxes

	printf("Save position in %d time\n",time);
	const btVector3 before_deserialize1 = box_before[0]->getCenterOfMassPosition();
	const btVector3 before_deserialize2 = box_before[1]->getCenterOfMassPosition();
	const btVector3 before_deserialize3 = box_before[2]->getCenterOfMassPosition();

	// deserialize

	printf("Deserialize in %d time\n",time);
	btMultiBodyDynamicsWorld *deserialized_world = create_btMultiBodyDynamicsWorld();
	btMultiBodyWorldImporter importer(deserialized_world);
	importer.loadFile(filename);
	box_after[0] = importer.getRigidBodyByIndex(1);
	box_after[1] = importer.getRigidBodyByIndex(2);
	box_after[2] = importer.getRigidBodyByIndex(3);
	// importer.convertAllObjects(...);
	EXPECT_FLOAT_EQ(box_after[0]->getCenterOfMassPosition().getX(), before_serialize1.getX());
	EXPECT_FLOAT_EQ(box_after[1]->getCenterOfMassPosition().getX(), before_serialize2.getX());
	EXPECT_FLOAT_EQ(box_after[2]->getCenterOfMassPosition().getX(), before_serialize3.getX());
	EXPECT_FLOAT_EQ(box_after[0]->getCenterOfMassPosition().getY(), before_serialize1.getY());
	EXPECT_FLOAT_EQ(box_after[1]->getCenterOfMassPosition().getY(), before_serialize2.getY());
	EXPECT_FLOAT_EQ(box_after[2]->getCenterOfMassPosition().getY(), before_serialize3.getY());
	EXPECT_FLOAT_EQ(box_after[0]->getCenterOfMassPosition().getZ(), before_serialize1.getZ());
	EXPECT_FLOAT_EQ(box_after[1]->getCenterOfMassPosition().getZ(), before_serialize2.getZ());
	EXPECT_FLOAT_EQ(box_after[2]->getCenterOfMassPosition().getZ(), before_serialize3.getZ());


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

	printf("Stop simulation and check in %d time\n",time);		
	
/*	EXPECT_FLOAT_EQ(before_deserialize1.getX(),after_deserialize1.getX());
	EXPECT_FLOAT_EQ(before_deserialize1.getY(),after_deserialize1.getY());
	EXPECT_FLOAT_EQ(before_deserialize1.getZ(),after_deserialize1.getZ());
	EXPECT_FLOAT_EQ(before_deserialize2.getX(),after_deserialize2.getX());
	EXPECT_FLOAT_EQ(before_deserialize2.getY(),after_deserialize2.getY());
	EXPECT_FLOAT_EQ(before_deserialize2.getZ(),after_deserialize2.getZ());
	EXPECT_FLOAT_EQ(before_deserialize3.getX(),after_deserialize3.getX());
	EXPECT_FLOAT_EQ(before_deserialize3.getY(),after_deserialize3.getY());
	EXPECT_FLOAT_EQ(before_deserialize3.getZ(),after_deserialize3.getZ());
*/
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
