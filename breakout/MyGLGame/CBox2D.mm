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

#define BRICK1_POS_X		100
#define BRICK1_POS_Y		1000
#define PADDLE_POS_X		400
#define PADDLE_POS_Y		50
#define BRICK_POS_X_SPACING 150
#define BRICK_POS_Y_SPACING 100
#define BRICK_WIDTH			100.0f
#define BRICK_HEIGHT		20.0f
#define BRICK_WAIT			1.5f
#define BALL_POS_X			400
#define BALL_POS_Y			100
#define BALL_RADIUS			15.0f
#define BALL_VELOCITY		100000000.0f
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
//        MyContact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
//        std::vector<MyContact>::iterator pos;
//        pos = std::find(_contacts.begin(), _contacts.end(), myContact);
//        if (pos != _contacts.end()) {
//            _contacts.erase(pos);
//        }
    };
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
    {
        MyContact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
        _contacts.push_back(myContact);
        
        b2WorldManifold worldManifold;
        contact->GetWorldManifold(&worldManifold);
        b2PointState state1[2], state2[2];
        b2GetPointStates(state1, state2, oldManifold, contact->GetManifold());
        if (state2[0] == b2_addState)
        {
            NSLog(@"HIT");
            // Use contact->GetFixtureA()->GetBody() to get the body
            b2Body* bodyA = contact->GetFixtureA()->GetBody();
            b2Body* bodyB = contact->GetFixtureB()->GetBody();
            bodyA->SetAwake(false);
            
            b2Vec2 currentVel = bodyB->GetLinearVelocity();
            
            
            //bodyB->SetLinearVelocity(b2Vec2(currentVel.x, -currentVel.y));
            
            //bodyB->SetLinearVelocity(b2Vec2(10000,0));
            //bodyB->SetAwake(false);
            CBox2D *parentObj = (__bridge CBox2D *)(bodyA->GetUserData());
            // Call RegisterHit (assume CBox2D object is in user data)
            
            //bodyToRemove = bodyA;
            
            [parentObj RegisterHit];
            
        }
    }
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
        MyContact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
        std::vector<MyContact>::iterator pos;
        pos = std::find(_contacts.begin(), _contacts.end(), myContact);
        if (pos != _contacts.end()) {
            _contacts.erase(pos);
        }
        
        b2Body* bodyA = contact->GetFixtureA()->GetBody();
        b2Body* bodyB = contact->GetFixtureB()->GetBody();
        bodyA->SetAwake(false);
    };
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
    b2Body *bricks[5], *paddle, *theBall, *leftWall, *rightWall, *topWall;
    CContactListener *contactListener;
    
    b2Body *_groundBody;
    b2Fixture *_bottomFixture;
    b2Fixture *_ballFixture;
    
    // GL-specific variables
    // You will need to set up 2 vertex arrays (for brick and ball)
    GLuint brickVertexArray[5], ballVertexArray, paddleVertexArray;
    int numBrickVerts, numBallVerts;
    GLKMatrix4 modelViewProjectionMatrix;

    // You will also need some extra variables here
    bool ballHitBrick;
    bool ballLaunched;
    bool gameStarted;
    float totalElapsedTime;
    
    int brickArrayLength;
    int gameScore;
    
}
@end

@implementation CBox2D

- (instancetype)init
{
    brickArrayLength = (sizeof(bricks)/sizeof(b2Body*));
    gameStarted = false;
    gameScore = 0;
    
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
        
        //[self createBoundingBox];
        
        float width = [UIScreen mainScreen].bounds.size.width;
        float height = [UIScreen mainScreen].bounds.size.height;
        
        b2BodyDef rightWallBodyDef;
        rightWallBodyDef.type = b2_staticBody;
        rightWallBodyDef.position.Set(width*2, height);
        
        rightWall = world->CreateBody(&rightWallBodyDef);
        if(rightWall){
            rightWall->SetUserData((__bridge void *)self);
            //secondBrick->SetUserData((__bridge void *)self);
            rightWall->SetAwake(true);
            //secondBrick->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(BRICK_HEIGHT/2, height);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.0f;
            fixtureDef.restitution = 1.0f;
            rightWall->CreateFixture(&fixtureDef);
        }
        
        b2BodyDef leftWallBodyDef;
        leftWallBodyDef.type = b2_staticBody;
        leftWallBodyDef.position.Set(0, height);
        
        leftWall = world->CreateBody(&leftWallBodyDef);
        if(leftWall){
            leftWall->SetUserData((__bridge void *)self);
            //secondBrick->SetUserData((__bridge void *)self);
            leftWall->SetAwake(true);
            //secondBrick->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(BRICK_HEIGHT/2, height);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.0f;
            fixtureDef.restitution = 1.0f;
            leftWall->CreateFixture(&fixtureDef);
        }
        
        b2BodyDef topWallBodyDef;
        topWallBodyDef.type = b2_staticBody;
        topWallBodyDef.position.Set(width, height*2);
        
        topWall = world->CreateBody(&topWallBodyDef);
        if(topWall){
            topWall->SetUserData((__bridge void *)self);
            //secondBrick->SetUserData((__bridge void *)self);
            topWall->SetAwake(true);
            //secondBrick->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(height, BRICK_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.0f;
            fixtureDef.restitution = 1.0f;
            topWall->CreateFixture(&fixtureDef);
        }
        
        
        b2BodyDef paddleBodyDef;
        paddleBodyDef.type = b2_staticBody;
        paddleBodyDef.position.Set(PADDLE_POS_X, PADDLE_POS_Y);
        
        paddle = world->CreateBody(&paddleBodyDef);
        
        if(paddle){
            paddle->SetUserData((__bridge void *)self);
            //secondBrick->SetUserData((__bridge void *)self);
            paddle->SetAwake(true);
            //secondBrick->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(BRICK_WIDTH/2, BRICK_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.0f;
            fixtureDef.restitution = 1.0f;
            paddle->CreateFixture(&fixtureDef);
        }
        

        for(int i=0; i<brickArrayLength; i++){
            // Set up the brick and ball objects for Box2D
            b2BodyDef brickBodyDef;
            brickBodyDef.type = b2_staticBody;
            brickBodyDef.position.Set(BRICK1_POS_X+i*BRICK_POS_X_SPACING, BRICK1_POS_Y);
            
            bricks[i] = world->CreateBody(&brickBodyDef);
        }
        
        
        //secondBrick =world->CreateBody(&brickBodyDef);
        
        for(int i=0; i<brickArrayLength; i++){
            if (bricks[i])//&&secondBrick)
            {
                bricks[i]->SetUserData((__bridge void *)self);
                //secondBrick->SetUserData((__bridge void *)self);
                bricks[i]->SetAwake(false);
                //secondBrick->SetAwake(false);
                b2PolygonShape dynamicBox;
                dynamicBox.SetAsBox(BRICK_WIDTH/2, BRICK_HEIGHT/2);
                b2FixtureDef fixtureDef;
                fixtureDef.shape = &dynamicBox;
                fixtureDef.density = 1.0f;
                fixtureDef.friction = 0.0f;
                fixtureDef.restitution = 1.0f;
                bricks[i]->CreateFixture(&fixtureDef);
                //secondBrick->CreateFixture(&fixtureDef);
            }
        }
        
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
            circleFixtureDef.friction = 0.0f;
            circleFixtureDef.restitution = 1.0f;
            theBall->CreateFixture(&circleFixtureDef);
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
    
    // Create edges around the entire screen
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0,0);
    b2Body *groundBody = world->CreateBody(&groundBodyDef);
    
    b2EdgeShape groundBox2;
    b2FixtureDef groundBoxDef;
    groundBoxDef.shape = &groundBox2;
    
    groundBox2.Set(b2Vec2(0,0), b2Vec2(width/PTM_RATIO, 0));
    _bottomFixture = groundBody->CreateFixture(&groundBoxDef);
    
    groundBox2.Set(b2Vec2(0,0), b2Vec2(0, height/PTM_RATIO));
    groundBody->CreateFixture(&groundBoxDef);
    
    groundBox2.Set(b2Vec2(0, height/PTM_RATIO), b2Vec2(width/PTM_RATIO,
                                                              height/PTM_RATIO));
    groundBody->CreateFixture(&groundBoxDef);
    
    groundBox2.Set(b2Vec2(width/PTM_RATIO, height/PTM_RATIO),
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
    float width = [UIScreen mainScreen].bounds.size.width;
    float height = [UIScreen mainScreen].bounds.size.height;
    
    //check that the ball has not slowed down
    
//    b2Vec2 velocity =theBall->GetLinearVelocity();
//    if(velocity.y!=BALL_VELOCITY&&velocity.y!=-BALL_VELOCITY){
//        if(velocity.y>0)
//            velocity.y = BALL_VELOCITY;
//        else if(velocity.y<0)
//            velocity.y = -BALL_VELOCITY;
//        
//        theBall->SetLinearVelocity(velocity);
//    }
    
    if(theBall->GetPosition().x>width*2){
        NSLog(@"right edge");
    }
    
    // Check here if we need to launch the ball
    //  and if so, use ApplyLinearImpulse() and SetActive(true)
    if (ballLaunched&&!gameStarted)
    {
        theBall->ApplyLinearImpulse(b2Vec2(0, BALL_VELOCITY), theBall->GetPosition(), true);
        
        theBall->SetActive(true);
#ifdef LOG_TO_CONSOLE
        NSLog(@"Applying impulse %f to ball\n", BALL_VELOCITY);
#endif
        ballLaunched = false;
        gameStarted = true;
    }
    
    // Check if it is time yet to drop the brick, and if so
    //  call SetAwake()
    totalElapsedTime += elapsedTime;
    //if ((totalElapsedTime > BRICK_WAIT) && firstBrick)
        //firstBrick->SetAwake(false);
    
    // If the last collision test was positive,
    //  stop the ball and destroy the brick
    if (ballHitBrick)
    {
        
        
        //theBall->SetAngularVelocity(0);
        //theBall->SetActive(true);
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
            
            if(bodyA==paddle||bodyB==paddle){
                NSLog(@"Paddle hit");
                [self doBounceAngle];
            }
            
            
            for(int i=0; i<brickArrayLength; i++){
                if(contact.fixtureA->GetBody()==bricks[i]){
                    bodyA = contact.fixtureA->GetBody();//Block
                }
                if(contact.fixtureB->GetBody()==bricks[i]){
                    bodyB = contact.fixtureB->GetBody();//Ball
                }
            }
            
//            if ((contact.fixtureA == _bottomFixture && contact.fixtureB == _ballFixture) ||
//                (contact.fixtureA == _ballFixture && contact.fixtureB == _bottomFixture)) {
//                NSLog(@"Ball hit bottom!");
//            }
        }
        if(bodyA!=nullptr){
            for(int i=0; i<brickArrayLength; i++){
                if(bodyA==bricks[i]){
                    gameScore+=10;
                    world->DestroyBody(bodyA);
                    bricks[i]=NULL;
                }
            }
        }
        if(bodyB!=nullptr){
            for(int i=0; i<brickArrayLength; i++){
                if(bodyB==bricks[i]){
                    gameScore+=10;
                    world->DestroyBody(bodyB);
                    bricks[i]=NULL;
                }
            }
        }
        
        //MyContact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
        //std::vector<MyContact>::iterator pos2;
        contactListener->_contacts.clear();
    }

   
    // Set up vertex arrays and buffers for the brick and ball here

    glEnable(GL_DEPTH_TEST);

    if (paddle)
    {
        glGenVertexArraysOES(1, &paddleVertexArray);
        glBindVertexArrayOES(paddleVertexArray);
        
        GLuint vertexBuffers[2];
        glGenBuffers(2, vertexBuffers);
        glBindBuffer(GL_ARRAY_BUFFER, paddleVertexArray);
        GLfloat vertPos[18];
        int k = 0;
        numBrickVerts = 0;
        vertPos[k++] = paddle->GetPosition().x - BRICK_WIDTH/2;
        vertPos[k++] = paddle->GetPosition().y + BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = paddle->GetPosition().x + BRICK_WIDTH/2;
        vertPos[k++] = paddle->GetPosition().y + BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = paddle->GetPosition().x + BRICK_WIDTH/2;
        vertPos[k++] = paddle->GetPosition().y - BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = paddle->GetPosition().x - BRICK_WIDTH/2;
        vertPos[k++] = paddle->GetPosition().y + BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = paddle->GetPosition().x + BRICK_WIDTH/2;
        vertPos[k++] = paddle->GetPosition().y - BRICK_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = paddle->GetPosition().x - BRICK_WIDTH/2;
        vertPos[k++] = paddle->GetPosition().y - BRICK_HEIGHT/2;
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
    
    for(int i=0; i<brickArrayLength; i++){
        if (bricks[i])
        {
            bricks[i]->SetAwake(false);
            glGenVertexArraysOES(1, &brickVertexArray[i]);
            glBindVertexArrayOES(brickVertexArray[i]);
            
            GLuint vertexBuffers[2];
            glGenBuffers(2, vertexBuffers);
            glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
            GLfloat vertPos[18];
            int k = 0;
            numBrickVerts = 0;
            vertPos[k++] = bricks[i]->GetPosition().x - BRICK_WIDTH/2;
            vertPos[k++] = bricks[i]->GetPosition().y + BRICK_HEIGHT/2;
            vertPos[k++] = 10;
            numBrickVerts++;
            vertPos[k++] = bricks[i]->GetPosition().x + BRICK_WIDTH/2;
            vertPos[k++] = bricks[i]->GetPosition().y + BRICK_HEIGHT/2;
            vertPos[k++] = 10;
            numBrickVerts++;
            vertPos[k++] = bricks[i]->GetPosition().x + BRICK_WIDTH/2;
            vertPos[k++] = bricks[i]->GetPosition().y - BRICK_HEIGHT/2;
            vertPos[k++] = 10;
            numBrickVerts++;
            vertPos[k++] = bricks[i]->GetPosition().x - BRICK_WIDTH/2;
            vertPos[k++] = bricks[i]->GetPosition().y + BRICK_HEIGHT/2;
            vertPos[k++] = 10;
            numBrickVerts++;
            vertPos[k++] = bricks[i]->GetPosition().x + BRICK_WIDTH/2;
            vertPos[k++] = bricks[i]->GetPosition().y - BRICK_HEIGHT/2;
            vertPos[k++] = 10;
            numBrickVerts++;
            vertPos[k++] = bricks[i]->GetPosition().x - BRICK_WIDTH/2;
            vertPos[k++] = bricks[i]->GetPosition().y - BRICK_HEIGHT/2;
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

    for(int i=0; i<brickArrayLength; i++){
        glBindVertexArrayOES(brickVertexArray[i]);
        if (bricks[i] && numBrickVerts > 0)
            glDrawArrays(GL_TRIANGLES, 0, numBrickVerts);
    }
    
    glBindVertexArrayOES(ballVertexArray);
    if (theBall && numBallVerts > 0)
        glDrawArrays(GL_TRIANGLE_FAN, 0, numBallVerts);
    
    glBindVertexArrayOES(paddleVertexArray);
    if (paddle && numBrickVerts > 0)
        glDrawArrays(GL_TRIANGLES, 0, numBrickVerts);
}

-(void)RegisterHit
{
    if(theBall->GetPosition().y<200)
        [self doBounceAngle];
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
    
    float width = [UIScreen mainScreen].bounds.size.width;
    float height = [UIScreen mainScreen].bounds.size.height;
    
    b2EdgeShape groundBox2;
    b2FixtureDef groundBoxDef;
    groundBoxDef.shape = groundBox;
    
    groundBox2.Set(b2Vec2(0,0), b2Vec2(width/PTM_RATIO, 0));
    _bottomFixture = groundBody->CreateFixture(&groundBoxDef);
    
    groundBox2.Set(b2Vec2(0,0), b2Vec2(0, height/PTM_RATIO));
    groundBody->CreateFixture(&groundBoxDef);
    
    groundBox2.Set(b2Vec2(0, height/PTM_RATIO), b2Vec2(width/PTM_RATIO,
                                                              height/PTM_RATIO));
    groundBody->CreateFixture(&groundBoxDef);
    
    groundBox2.Set(b2Vec2(width/PTM_RATIO, height/PTM_RATIO),
                  b2Vec2(width/PTM_RATIO, 0));
    groundBody->CreateFixture(&groundBoxDef);
    
    
    
    
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

-(void) movePlayer: (float)move
{
    paddle->SetTransform(paddle->GetPosition() + b2Vec2(move, 0), paddle->GetAngle());
    //Set player boundry
    if(paddle->GetPosition().x >= 500){
        
        
    }
    if(paddle->GetPosition().x <= 200){
        
    }
    
    /*  if(theBall->GetPosition().y <= 100){
     
     theBall->SetTransform(b2Vec2(BALL_POS_X, BALL_POS_Y), 0);
     
     }*/
}

-(void)doBounceAngle{
    NSLog(@"bounce angle");
    float xdiff = theBall->GetPosition().x - paddle->GetPosition().x;
    theBall->ApplyLinearImpulse(b2Vec2(xdiff*5000, 0), theBall->GetPosition(), true);
    //theBall->ApplyForceToCenter(b2Vec2(xdiff*1000000000000, 0), true);
}

@end
