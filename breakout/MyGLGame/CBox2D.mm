//
//  CBox2D.m
//  MyGLGame
//
//  Created by Borna Noureddin on 2015-03-17.
//  Copyright (c) 2015 BCIT. All rights reserved.
//

#include <Box2D/Box2D.h>
#include "CBox2D.h"
#include <OpenGLES/ES2/glext.h>
#include <stdio.h>
#import <vector>

#define BUFFER_OFFSET(i) ((char *)NULL + (i))
//#define LOG_TO_CONSOLE



#pragma mark - Brick and ball physics parameters

// Set up brick and ball physics parameters here:
//   position, width+height (or radius), velocity,
//   and how long to wait before dropping brick

#define BRICK1_POS_X		400
#define BRICK1_POS_Y		500
#define BRICK2_POS_X		400
#define BRICK2_POS_Y		100
#define BRICK_WIDTH			100.0f
#define BRICK_HEIGHT		10.0f
#define BRICK_WAIT			1.5f
#define BALL_POS_X			400
#define BALL_POS_Y			50
#define BALL_RADIUS			15.0f
#define BALL_VELOCITY		100000.0f
#define BALL_SPHERE_SEGS	128

#define PTM_RATIO 32.0

const float MAX_TIMESTEP = 1.0f/60.0f;
const int NUM_VEL_ITERATIONS = 10;
const int NUM_POS_ITERATIONS = 3;


#pragma mark - Box2D contact listener class

struct MyContact {
    b2Fixture *fixtureA;
    b2Fixture *fixtureB;
    bool operator==(const MyContact& other) const
    {
        return (fixtureA == other.fixtureA) && (fixtureB == other.fixtureB);
    }
};

class CContactListener : public b2ContactListener
{
    
    
public:
    std::vector<MyContact>_contacts;
    b2Body* bodyToRemove = nullptr;
    
    void BeginContact(b2Contact* contact) {
        MyContact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
        _contacts.push_back(myContact);
    };
    void EndContact(b2Contact* contact) {
        MyContact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
        std::vector<MyContact>::iterator pos;
        pos = std::find(_contacts.begin(), _contacts.end(), myContact);
        if (pos != _contacts.end()) {
            _contacts.erase(pos);
        }
    };
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
    {
        b2WorldManifold worldManifold;
        contact->GetWorldManifold(&worldManifold);
        b2PointState state1[2], state2[2];
        b2GetPointStates(state1, state2, oldManifold, contact->GetManifold());
        if (state2[0] == b2_addState)
        {
            // Use contact->GetFixtureA()->GetBody() to get the body
            b2Body* bodyA = contact->GetFixtureA()->GetBody();
            b2Body* bodyB = contact->GetFixtureB()->GetBody();
            bodyA->SetAwake(false);
            bodyA->SetLinearVelocity(b2Vec2(10000,0));
            bodyB->SetAwake(false);
            CBox2D *parentObj = (__bridge CBox2D *)(bodyA->GetUserData());
            // Call RegisterHit (assume CBox2D object is in user data)
            
            bodyToRemove = bodyA;
            
            [parentObj RegisterHit];
            
        }
    }
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {};
};


#pragma mark - CBox2D

@interface CBox2D ()
{
    // Box2D-specific objects
    b2Vec2 *gravity;
    b2World *world;
    b2BodyDef *groundBodyDef;
    b2Body *groundBody;
    b2PolygonShape *groundBox;
    b2Body *firstBrick, *secondBrick, *theBall;
    CContactListener *contactListener;
    
    b2Fixture *_bottomFixture;
    
    // GL-specific variables
    // You will need to set up 2 vertex arrays (for brick and ball)
    GLuint brickVertexArray[5], ballVertexArray;
    int numBrickVerts, numBallVerts;
    GLKMatrix4 modelViewProjectionMatrix;

    // You will also need some extra variables here
    bool ballHitBrick;
    bool ballLaunched;
    float totalElapsedTime;
    
}
@end

@implementation CBox2D

- (instancetype)init
{
    self = [super init];
    if (self) {
        gravity = new b2Vec2(0.0f, 0.0f);
        world = new b2World(*gravity);
        
        // For HelloWorld
        groundBodyDef = NULL;
        groundBody = NULL;
        groundBox = NULL;

        // For brick & ball sample
        contactListener = new CContactListener();
        world->SetContactListener(contactListener);
        
        // Set up the brick and ball objects for Box2D
        b2BodyDef brickBodyDef;
        brickBodyDef.type = b2_dynamicBody;
        brickBodyDef.position.Set(BRICK1_POS_X, BRICK1_POS_Y);
        firstBrick = world->CreateBody(&brickBodyDef);
        brickBodyDef.position.Set(BRICK2_POS_X, BRICK2_POS_Y);
        //secondBrick =world->CreateBody(&brickBodyDef);
        if (firstBrick)//&&secondBrick)
        {
            firstBrick->SetUserData((__bridge void *)self);
            //secondBrick->SetUserData((__bridge void *)self);
            firstBrick->SetAwake(false);
            //secondBrick->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(BRICK_WIDTH/2, BRICK_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 1.0f;
            firstBrick->CreateFixture(&fixtureDef);
            //secondBrick->CreateFixture(&fixtureDef);
            
            b2BodyDef ballBodyDef;
            ballBodyDef.type = b2_dynamicBody;
            ballBodyDef.position.Set(BALL_POS_X, BALL_POS_Y);
            theBall = world->CreateBody(&ballBodyDef);
            if (theBall)
            {
                theBall->SetUserData((__bridge void *)self);
                theBall->SetAwake(false);
                b2CircleShape circle;
                circle.m_p.Set(0, 0);
                circle.m_radius = BALL_RADIUS;
                b2FixtureDef circleFixtureDef;
                circleFixtureDef.shape = &circle;
                circleFixtureDef.density = 1.0f;
                circleFixtureDef.friction = 0.3f;
                circleFixtureDef.restitution = 1.0f;
                theBall->CreateFixture(&circleFixtureDef);
            }
        }
        
        
        
        totalElapsedTime = 0;
        ballHitBrick = false;
        ballLaunched = false;
    }
    return self;
}

-(void)createBoundingBox{
    float width = [UIScreen mainScreen].bounds.size.width;
    float height = [UIScreen mainScreen].bounds.size.height;
    
    //CGSize winSize = [CCDirector sharedDirector].winSize;
    
    // Create a world
    world = new b2World(*(gravity));
    
    // Create edges around the entire screen
    //b2BodyDef groundBodyDef;
//    groundBodyDef = new b2BodyDef;
//    groundBodyDef->position.Set(0.0f,0.0f);
//    groundBody = world->CreateBody(groundBodyDef);
    
    b2EdgeShape groundBoundBox;
    b2FixtureDef groundBoxDef;
    groundBoxDef.shape = &groundBoundBox;
    
    groundBoundBox.Set(b2Vec2(0.0f,0.0f), b2Vec2(width/PTM_RATIO, 0.0f));
    _bottomFixture = groundBody->CreateFixture(&groundBoxDef);
    
    groundBoundBox.Set(b2Vec2(0,0), b2Vec2(0, height/PTM_RATIO));
    groundBody->CreateFixture(&groundBoxDef);
    
    groundBoundBox.Set(b2Vec2(0, height/PTM_RATIO), b2Vec2(width/PTM_RATIO,
                                                              height/PTM_RATIO));
    groundBody->CreateFixture(&groundBoxDef);
    
    groundBoundBox.Set(b2Vec2(width/PTM_RATIO, height/PTM_RATIO),
                  b2Vec2(width/PTM_RATIO, 0));
    groundBody->CreateFixture(&groundBoxDef);

}

- (void)dealloc
{
    if (gravity) delete gravity;
    if (world) delete world;
    if (groundBodyDef) delete groundBodyDef;
    if (groundBox) delete groundBox;
    if (contactListener) delete contactListener;
}

-(void)Update:(float)elapsedTime
{
    // Check here if we need to launch the ball
    //  and if so, use ApplyLinearImpulse() and SetActive(true)
    if (ballLaunched)
    {
        theBall->ApplyLinearImpulse(b2Vec2(0, BALL_VELOCITY), theBall->GetPosition(), true);
        
        theBall->SetActive(true);
#ifdef LOG_TO_CONSOLE
        NSLog(@"Applying impulse %f to ball\n", BALL_VELOCITY);
#endif
        ballLaunched = false;
    }
    
    // Check if it is time yet to drop the brick, and if so
    //  call SetAwake()
    totalElapsedTime += elapsedTime;
    if ((totalElapsedTime > BRICK_WAIT) && firstBrick)
        firstBrick->SetAwake(false);
    
    // If the last collision test was positive,
    //  stop the ball and destroy the brick
    if (ballHitBrick)
    {
        b2Vec2 currentVel = theBall->GetLinearVelocity();
        //theBall->SetLinearVelocity(b2Vec2(currentVel.x, -BALL_VELOCITY));
        
        theBall->SetAngularVelocity(0);
        theBall->SetActive(true);
        //world->DestroyBody(firstBrick);
        //firstBrick = NULL;
        ballHitBrick = false;
    }
    
//    if(contactListener->bodyToRemove!=nullptr){
//        b2Body* toRemove = (b2Body *)(contactListener->bodyToRemove);
//        world->DestroyBody(toRemove);
//    }

    if (world)
    {
        while (elapsedTime >= MAX_TIMESTEP)
        {
            world->Step(MAX_TIMESTEP, NUM_VEL_ITERATIONS, NUM_POS_ITERATIONS);
            elapsedTime -= MAX_TIMESTEP;
        }
        
        if (elapsedTime > 0.0f)
        {
            world->Step(elapsedTime, NUM_VEL_ITERATIONS, NUM_POS_ITERATIONS);
        }
        
        std::vector<MyContact>::iterator pos;
        b2Body *bodyA=nullptr;
        b2Body *bodyB=nullptr;
        for(pos = contactListener->_contacts.begin();
            pos != contactListener->_contacts.end(); ++pos) {
            MyContact contact = *pos;
            
            bodyA = contact.fixtureA->GetBody();//Block
            bodyB = contact.fixtureB->GetBody();//Ball
            
            
            
//            if ((contact.fixtureA == _bottomFixture && contact.fixtureB == _ballFixture) ||
//                (contact.fixtureA == _ballFixture && contact.fixtureB == _bottomFixture)) {
//                NSLog(@"Ball hit bottom!");
//            }
        }
        if(bodyA!=nullptr)
        world->DestroyBody(bodyA);
    }

   
    // Set up vertex arrays and buffers for the brick and ball here

    glEnable(GL_DEPTH_TEST);

    if (firstBrick)
    {
        glGenVertexArraysOES(1, &brickVertexArray[0]);
        glBindVertexArrayOES(brickVertexArray[0]);
        
        GLuint vertexBuffers[2];
        glGenBuffers(2, vertexBuffers);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
        GLfloat vertPos[18];
        int k = 0;
        numBrickVerts = 0;
        vertPos[k++] = firstBrick->GetPosition().x - BRICK_WIDTH/2;
        vertPos[k++] = firstBrick->GetPosition().y + BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = firstBrick->GetPosition().x + BRICK_WIDTH/2;
        vertPos[k++] = firstBrick->GetPosition().y + BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = firstBrick->GetPosition().x + BRICK_WIDTH/2;
        vertPos[k++] = firstBrick->GetPosition().y - BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = firstBrick->GetPosition().x - BRICK_WIDTH/2;
        vertPos[k++] = firstBrick->GetPosition().y + BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = firstBrick->GetPosition().x + BRICK_WIDTH/2;
        vertPos[k++] = firstBrick->GetPosition().y - BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = firstBrick->GetPosition().x - BRICK_WIDTH/2;
        vertPos[k++] = firstBrick->GetPosition().y - BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertPos), vertPos, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribPosition);
        glVertexAttribPointer(VertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        GLfloat vertCol[numBrickVerts*3];
        for (k=0; k<numBrickVerts*3; k+=3)
        {
            vertCol[k] = 1.0f;
            vertCol[k+1] = 0.0f;
            vertCol[k+2] = 0.0f;
        }
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertCol), vertCol, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribColor);
        glVertexAttribPointer(VertexAttribColor, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        glBindVertexArrayOES(0);
    }

    
    if (theBall)
    {
        glGenVertexArraysOES(1, &ballVertexArray);
        glBindVertexArrayOES(ballVertexArray);
        
        GLuint vertexBuffers[2];
        glGenBuffers(2, vertexBuffers);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
        GLfloat vertPos[3*(BALL_SPHERE_SEGS+2)];
        int k = 0;
        vertPos[k++] = theBall->GetPosition().x;
        vertPos[k++] = theBall->GetPosition().y;
        vertPos[k++] = 0;
        numBallVerts = 1;
        for (int n=0; n<=BALL_SPHERE_SEGS; n++)
        {
            float const t = 2*M_PI*(float)n/(float)BALL_SPHERE_SEGS;
            vertPos[k++] = theBall->GetPosition().x + sin(t)*BALL_RADIUS;
            vertPos[k++] = theBall->GetPosition().y + cos(t)*BALL_RADIUS;
            vertPos[k++] = 0;
            numBallVerts++;
        }
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertPos), vertPos, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribPosition);
        glVertexAttribPointer(VertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        GLfloat vertCol[numBallVerts*3];
        for (k=0; k<numBallVerts*3; k+=3)
        {
            vertCol[k] = 0.0f;
            vertCol[k+1] = 1.0f;
            vertCol[k+2] = 0.0f;
        }
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertCol), vertCol, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribColor);
        glVertexAttribPointer(VertexAttribColor, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        glBindVertexArrayOES(0);
    }

    // For now assume simple ortho projection since it's only 2D
    GLKMatrix4 projectionMatrix = GLKMatrix4MakeOrtho(0, 800, 0, 1500, -10, 100);
    GLKMatrix4 modelViewMatrix = GLKMatrix4Identity;
    modelViewProjectionMatrix = GLKMatrix4Multiply(projectionMatrix, modelViewMatrix);
}

-(void)Render:(int)mvpMatPtr
{
#ifdef LOG_TO_CONSOLE
    if (theBall)
        printf("Ball: (%5.3f,%5.3f)\t",
               theBall->GetPosition().x, theBall->GetPosition().y);
    if (theBrick)
        printf("Brick: (%5.3f,%5.3f)",
               theBrick->GetPosition().x, theBrick->GetPosition().y);
    printf("\n");
#endif
    
    glClearColor(0, 0, 0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glUniformMatrix4fv(mvpMatPtr, 1, 0, modelViewProjectionMatrix.m);

    // Bind each vertex array and call glDrawArrays
    //  for each of the ball and brick

    glBindVertexArrayOES(brickVertexArray[0]);
    if (firstBrick && numBrickVerts > 0)
        glDrawArrays(GL_TRIANGLES, 0, numBrickVerts);
    
    glBindVertexArrayOES(ballVertexArray);
    if (theBall && numBallVerts > 0)
        glDrawArrays(GL_TRIANGLE_FAN, 0, numBallVerts);
}

-(void)RegisterHit
{
    // Set some flag here for processing later...
    ballHitBrick = true;
}

-(void)removeBlock:(void*)body{
    b2Body* gameBody = (b2Body*)body;
    //world->DestroyBody(gameBody);
}

-(void)LaunchBall
{
    // Set some flag here for processing later...
    ballLaunched = true;
}



-(void)HelloWorld
{
    groundBodyDef = new b2BodyDef;
    groundBodyDef->position.Set(0.0f, -10.0f);
    groundBody = world->CreateBody(groundBodyDef);
    groundBox = new b2PolygonShape;
    groundBox->SetAsBox(50.0f, 10.0f);
    
    groundBody->CreateFixture(groundBox, 0.0f);
    
    // Define the dynamic body. We set its position and call the body factory.
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(0.0f, 4.0f);
    b2Body* body = world->CreateBody(&bodyDef);
    
    // Define another box shape for our dynamic body.
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);
    
    // Define the dynamic body fixture.
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    
    // Set the box density to be non-zero, so it will be dynamic.
    fixtureDef.density = 1.0f;
    
    // Override the default friction.
    fixtureDef.friction = 0.0f;
    
    // Add the shape to the body.
    body->CreateFixture(&fixtureDef);
    
    //[self createBoundingBox];
    
    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float32 timeStep = 1.0f / 60.0f;
    int32 velocityIterations = 6;
    int32 positionIterations = 2;
    
    // This is our little game loop.
    for (int32 i = 0; i < 60; ++i)
    {
        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world->Step(timeStep, velocityIterations, positionIterations);
        
        // Now print the position and angle of the body.
        b2Vec2 position = body->GetPosition();
        float32 angle = body->GetAngle();
        
        printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
    }
    
    //theBall->SetLinearVelocity(b2Vec2(10000, BALL_VELOCITY));
}

@end
