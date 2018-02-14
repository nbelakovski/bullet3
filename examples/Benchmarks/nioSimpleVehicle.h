//
// Created by nickolai.belakovski on 2/8/18.
//

#ifndef BULLET_PHYSICS_NIOSIMPLEVEHICLE_H
#define BULLET_PHYSICS_NIOSIMPLEVEHICLE_H

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <btBulletDynamicsCommon.h>
#include <array>
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"


// This file mostly copied from ForkLiftDemo.h/cpp

btCompoundShape * createChassis()
{
    // I dislike how this isn't an idempotent operation, since it creates new memory every time, but oh well. Maybe fix it later.
    btCompoundShape * compoundShape = new btCompoundShape();
    btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
    btTransform localTransform;
    localTransform.setIdentity();
    localTransform.setOrigin(btVector3(btScalar(0), btScalar(1), btScalar(0)));
    compoundShape->addChildShape(localTransform, chassisShape);

    // add suppShape - is this suspension? It might be another part of the forklift, maybe unnecessary for this
//    btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f,0.1f,0.5f));
//    localTransform.setOrigin(btVector3(btScalar(0), btScalar(1), btScalar(2.5)));
//    compoundShape->addChildShape(localTransform, suppShape); // it would be nice if unique pointers were used :(
    return compoundShape;
}

btRigidBody * createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

    btRigidBody* body = new btRigidBody(cInfo);

    return body;
}

// not purely without side effects either, since it creates a wheel shape. Maybe this should be put into a class that
// keeps track of its created objects or something. Way too much dynamic memory allocation going on around here
std::array<int, 4> createWheelGraphicTargets(GUIHelperInterface * guiHelper, const float wheelWidth, const float wheelRadius)
{
    btCylinderShape * wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));

    guiHelper->createCollisionShapeGraphicsObject(wheelShape);
    int wheelGraphicsIndex = wheelShape->getUserIndex();

    const float position[4]={0,10,10,0};
    const float quaternion[4]={0,0,0,1};
    const float color[4]={0,1,0,1};
    const float scaling[4] = {1,1,1,1};

    std::array<int, 4> wheelInstances = {0};
    for (auto & wheelInstance : wheelInstances)
    {
        wheelInstance = guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color, scaling);
    }
    return wheelInstances;
}

void setVehicleWheelConstants(btRaycastVehicle * vehicle)
{
    const float wheelFriction = 1000;
    const float suspensionStiffness = 20.f;
    const float suspensionDamping = 2.3f;
    const float suspensionCompression = 4.4f;
    const float rollInfluence = 0.1f;
    for (int i=0; i < vehicle->getNumWheels();i++)
    {
        btWheelInfo& wheel = vehicle->getWheelInfo(i);
        wheel.m_suspensionStiffness = suspensionStiffness;
        wheel.m_wheelsDampingRelaxation = suspensionDamping;
        wheel.m_wheelsDampingCompression = suspensionCompression;
        wheel.m_frictionSlip = wheelFriction;
        wheel.m_rollInfluence = rollInfluence;
    }
}

class nioSimpleVehicle
{
private:
    btVehicleRaycaster*	m_vehicleRayCaster;
    btRaycastVehicle*	m_vehicle;
    std::array<int, 4> m_wheelInstances = {-1}; // for graphics
public:
    btRaycastVehicle * getVehicle() {return m_vehicle;}

    void render(GUIHelperInterface * guiHelper)
    {
        // create the wheel instance if they don't exist
        if(m_wheelInstances[0] == -1)
        {
            const float wheelWidth = 0.4f;
            const float wheelRadius = 0.5f;
            m_wheelInstances = createWheelGraphicTargets(guiHelper, wheelWidth, wheelRadius);
        }
        for (int i=0;i<m_vehicle->getNumWheels();i++)
        {
            //synchronize the wheels with the (interpolated) chassis worldtransform
            m_vehicle->updateWheelTransform(i,true);

            CommonRenderInterface* renderer = guiHelper->getRenderInterface();
            if (renderer)
            {
                btTransform tr = m_vehicle->getWheelInfo(i).m_worldTransform;
                btVector3 pos=tr.getOrigin();
                btQuaternion orn = tr.getRotation();
                renderer->writeSingleInstanceTransformToCPU(pos,orn,m_wheelInstances[i]);
            }
        }

    }

    nioSimpleVehicle(btDiscreteDynamicsWorld* dynamicsWorld)
    {
        // create the chassis shape
        btCompoundShape * compoundShape = createChassis();

        // add that shape to the world at the origin, and give it some mass
        btScalar mass(100);
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.setOrigin(btVector3(0,45,0));
        btRigidBody * body = createRigidBody(mass, startTransform, compoundShape);
        body->setUserIndex2(1); // TODO: need some sort of global id generation. Check in serializer, it might have something
        body->setActivationState(DISABLE_DEACTIVATION); // never deactivate the chassis

        // new create the vehicle?

        m_vehicleRayCaster = new btDefaultVehicleRaycaster(dynamicsWorld);
        btRaycastVehicle::btVehicleTuning tuning; // used for wheels, but not for vehicle. Maybe wheels should get it from vehicle?
        m_vehicle = new btRaycastVehicle(tuning, body, m_vehicleRayCaster);

        dynamicsWorld->addRigidBody(m_vehicle->getRigidBody());
        dynamicsWorld->addVehicle(m_vehicle);

        // Todo: move wheel generation into its own function
        const int rightIndex = 0;
        const int upIndex = 1;
        const int forwardIndex = 2;
        m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

        btVector3 wheelAxleCS(-1,0,0);
        btVector3 wheelDirectionCS0(0,-1,0);
        const float connectionHeight = 1.2f;
        const int CUBE_HALF_EXTENTS = 1;
        const float wheelWidth = 0.4f; // todo: these should be consistent with the ones that are rendered
        const float wheelRadius = 0.4f;
        const btScalar suspensionRestLength(0.6);

        bool isFrontWheel=true;
        btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,tuning,isFrontWheel);

        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);
        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,tuning,isFrontWheel);

        isFrontWheel = false;

        connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,tuning,isFrontWheel);

        connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
        m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,tuning,isFrontWheel);

        // set all wheel constants
        setVehicleWheelConstants(m_vehicle);

        // original code does some sort of reset. Copying that stuff here
        m_vehicle->resetSuspension();

    }


};

#endif //BULLET_PHYSICS_NIOSIMPLEVEHICLE_H
