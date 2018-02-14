/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 50
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletWorldImporter/btBulletWorldImporter.h"

#include <iostream>

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample(){}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}
	bool m_load;

	virtual void processCommandLineArgs(int argc, char * argv[])
	{
		m_load = (argc > 1);
	}
};

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	// m_broadphase->resetPool(m_dispatcher);
	// m_solver->reset();
	m_dynamicsWorld->setGravity(btVector3(0,-1,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	if (false)
	{
		btBulletWorldImporter* loader = new btBulletWorldImporter(m_dynamicsWorld);
		// loader->setVerboseMode(1);
		loader->loadFile("basic_demo.bullet");
	}
	else
	{

		///create a few basic rigid bodies
		btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
		

		//groundShape->initializePolyhedralFeatures();
		//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
		
		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-50,0));

		{
			btScalar mass(0.);
			createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
		}


		{
			//create a few dynamic rigidbodies
			// Re-using the same collision is better for memory usage and performance

			btBoxShape* colShape = createBoxShape(btVector3(.1,.1,.1));
			

			//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
			m_collisionShapes.push_back(colShape);

			/// Create Dynamic Objects
			btTransform startTransform;
			startTransform.setIdentity();

			btScalar	mass(1.f);

			//rigidbody is dynamic if and only if mass is non zero, otherwise static
			bool isDynamic = (mass != 0.f);

			btVector3 localInertia(0,0,0);
			if (isDynamic)
				colShape->calculateLocalInertia(mass,localInertia);


			// m_broadphase->resetPool(m_dispatcher);
			// m_solver->reset();
			for (int k=0;k<ARRAY_SIZE_Y;k++)
			{
				for (int i=0;i<ARRAY_SIZE_X;i++)
				{
					for(int j = 0;j<ARRAY_SIZE_Z;j++)
					{
						startTransform.setOrigin(btVector3(
											btScalar(0.25*i),
											btScalar(1.5+.2*k),
											btScalar(0.25*j)));


						btRigidBody* body = createRigidBody(mass,startTransform,colShape);
						int id = k * ARRAY_SIZE_X * ARRAY_SIZE_Z + i * ARRAY_SIZE_Z + j;
						body->setUserIndex2(id);
					}
				}
			}
            // simple 3 block simulation:
//            startTransform.setOrigin(btVector3(btScalar(.25), btScalar(1), btScalar(.25)));
//            // also want to add velocity upwards on this guy
// 			btRigidBody* body = createRigidBody(mass,startTransform,colShape);
//            body->setUserIndex2(5);
//            body->setLinearVelocity(btVector3(btScalar(.2), btScalar(3), btScalar(0)));
//            body->setAngularVelocity(btVector3(btScalar(1), btScalar(1), btScalar(1)));
//
//            startTransform.setOrigin(btVector3(btScalar(.4), btScalar(2), btScalar(.2)));
//            body = createRigidBody(mass,startTransform,colShape);
//            body->setUserIndex2(7);
//            startTransform.setOrigin(btVector3(btScalar(.4), btScalar(2.2), btScalar(.2)));
//            body = createRigidBody(mass,startTransform,colShape);
//            body->setUserIndex2(8);
		}
	}
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	
}


void BasicExample::renderScene()
{
	static int counter = 0;
	++counter;
	CommonRigidBodyBase::renderScene();
	for (int i = 0; i < m_dynamicsWorld->getCollisionObjectArray().size(); ++i)
	{
        int id = m_dynamicsWorld->getCollisionObjectArray()[i]->getUserIndex2();
		if ( id== 5 || id == 7)
		{
			btTransform t = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform();
			std::cout << counter << ":" << id << ":" << t.getOrigin().getX() << ", " << t.getOrigin().getY() << ", " << t.getOrigin().getZ() << std::endl;
//			break;
		}
		btTransform t = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform();
	}
	// if (m_dynamicsWorld->m_localTime > 8.2e-7)
	// {
	// 	std::cout << "Final positions:" << std::endl;
	// 	for (int i = 0; i < m_dynamicsWorld->getCollisionObjectArray().size(); ++i)
	// 	{
	// 		btTransform t = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform();
	// 		std::cout << m_dynamicsWorld->m_localTime << ":" << t.getOrigin().getX() << ", " << t.getOrigin().getY() << ", " << t.getOrigin().getZ() << std::endl;
	// 	}
	// 	std::cout << "My work here is done. " << m_dynamicsWorld->getCollisionObjectArray().size() << std::endl;
	// 	// exit(0);
	// }
	static bool saved = true;
	if (counter == 200 && !saved)
	{
		std::cout << "Saving..." << std::endl;	
		btDefaultSerializer* s = new btDefaultSerializer();
		s->setSerializationFlags(BT_SERIALIZE_CONTACT_MANIFOLDS);
		m_dynamicsWorld->serialize(s);
		FILE* file = fopen("basic_demo.bullet", "wb");
		fwrite(s->getBufferPointer(), s->getCurrentBufferSize(), 1, file);
		fclose(file);
		saved = true;
	}
	static bool loaded = false;
	if (m_load && saved && !loaded)
	{
		std::cout << "Loading..." << std::endl;
		btDiscreteDynamicsWorld * m_dynamicsWorldCopy = m_dynamicsWorld;
		// delete m_dynamicsWorld;
		m_guiHelper->removeAllGraphicsInstances();
		createEmptyDynamicsWorld();
		m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

		if (m_dynamicsWorld->getDebugDrawer())
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

		btBulletWorldImporter* loader = new btBulletWorldImporter(m_dynamicsWorld);
		loader->loadFile("basic_demo.bullet");
		loaded = true;
		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
		delete loader;
	}
}







CommonExampleInterface*    BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)



