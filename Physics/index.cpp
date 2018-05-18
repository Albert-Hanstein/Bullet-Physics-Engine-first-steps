/*
 * This is a very simple example of using Bullet
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "World.h"

// FILE *flog;

const double PI = 3.141592653589793;
const double PIo2 = PI / 2.;
const double PIo4 = PI / 4.;
const double PI2 = PI * 2.;
const float lod = PI / 32.;

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "VertexBufferObject.h"
#include "glslprogram.h"
#include "stb_image.h"
#include "utils.h"
/*
 * Set up bullet - globals.
 */
#include <btBulletDynamicsCommon.h>

struct Vertex {
  Vertex(): color{1,1,1} {};
  GLfloat position[3];
  GLfloat color[3];
};

typedef struct{
  Vertex p1, p2, p3;
} Facet;

btBroadphaseInterface* broadphase;
btDefaultCollisionConfiguration* collisionConfiguration;
btCollisionDispatcher* dispatcher;
btSequentialImpulseConstraintSolver* solver;
btDiscreteDynamicsWorld* dynamicsWorld;

std::vector<btRigidBody*> MovingBits; // so that can get at all bits
std::vector<btRigidBody*> StaticBits; // especially during clean up.

/* Return the midpoint of two vectors */
Vertex Midpoint(Vertex p1, Vertex p2){
  Vertex p;
  p.position[0] = (p1.position[0] + p2.position[0])/2;
  p.position[1] = (p1.position[1] + p2.position[1])/2;
  p.position[2] = (p1.position[2] + p2.position[2])/2;
  return p;
}
/* Normalise a vector */
void Normalise(Vertex *p){
  float length;
  length = sqrt(pow(p->position[0],2) + pow(p->position[1],2) + pow(p->position[2],2));
  if(length != 0){
    p->position[0] /= length;
    p->position[1] /= length;
    p->position[2] /= length;
  } else{
    p->position[0] = 0;
    p->position[1] = 0;
    p->position[2] = 0;
  }
}

/* Calculate and return the number of vertices in our sphere */
int CreateUnitSphere(int iterations, Facet *facets){
  int i, j, n, nstart;
  Vertex p1,p2,p3,p4,p5,p6;
  p1.position[0] = 0.0; p1.position[1] = 0.0; p1.position[2] = 5.0;
  p2.position[0] = 0.0; p2.position[1] = 0.0; p2.position[2] = -5.0;
  p3.position[0] = -5.0; p3.position[1] = -5.0; p3.position[2] = 0.0;
  p4.position[0] = 5.0; p4.position[1] = -5.0; p4.position[2] = 0.0;
  p5.position[0] = 5.0; p5.position[1] = 5.0; p5.position[2] = 0.0;
  p6.position[0] = -5.0; p4.position[1] = -5.0; p4.position[2] = 0.0;
  Normalise(&p1); Normalise(&p2); Normalise(&p3); Normalise(&p4); Normalise(&p5); Normalise(&p6);

  facets[0].p1 = p1;facets[0].p2 = p4;facets[0].p3 = p5;
  facets[1].p1 = p1;facets[1].p2 = p5;facets[1].p3 = p6;
  facets[2].p1 = p1;facets[2].p2 = p6;facets[2].p3 = p3;
  facets[3].p1 = p1;facets[3].p2 = p3;facets[3].p3 = p4;
  facets[4].p1 = p2;facets[4].p2 = p5;facets[4].p3 = p4;
  facets[5].p1 = p2;facets[5].p2 = p6;facets[5].p3 = p5;
  facets[6].p1 = p2;facets[6].p2 = p3;facets[6].p3 = p6;
  facets[7].p1 = p2;facets[7].p2 = p4;facets[7].p3 = p3;

  n = 8;

  for(i = 1; i<iterations; i++){
    nstart = n;

    for(j = 0; j<nstart; j++){
      /* Create initial copies for the new facets */
      facets[n] = facets[j];
      facets[n+1] = facets[j];
      facets[n+2] = facets[j];

      /* Calculate the midpoints */
      p1 = Midpoint(facets[j].p1, facets[j].p2);
      p2 = Midpoint(facets[j].p2, facets[j].p3);
      p3 = Midpoint(facets[j].p3, facets[j].p1);

      /* Replace the current facet */
      facets[j].p2 = p1;
      facets[j].p3 = p3;

      /* Create the changed vertices in the new facets */
      facets[n].p1 = p1;
      facets[n].p3 = p2;
      facets[n+1].p1 = p3;
      facets[n+1].p2 = p2;
      facets[n+2].p1 = p1;
      facets[n+2].p2 = p2;
      facets[n+2].p3 = p3;
      n += 3;
    }
  }
  for(j = 0; j<n; j++){
    Normalise(&facets[j].p1);
    Normalise(&facets[j].p2);
    Normalise(&facets[j].p3);
  }
  return(n);
}

/*
 * Bullet Code
 */
btRigidBody* SetSphere(float size, btTransform T) {
  btCollisionShape* fallshape = new btSphereShape(size);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(T);
  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallshape->calculateLocalInertia(mass,fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallshape,fallInertia);
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
  fallRigidBody->setLinearVelocity(btVector3(-5, 20, 0));
  fallRigidBody->setRestitution(COE);
  dynamicsWorld->addRigidBody(fallRigidBody);
  return fallRigidBody;
}

btRigidBody* SetCone(float radius, btTransform T) {
  btCollisionShape* fallshape = new btConeShape(radius, 12.0);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(T);
  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallshape->calculateLocalInertia(mass,fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallshape,fallInertia);
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
  fallRigidBody->setLinearVelocity(btVector3(-5, 20, 0));
  fallRigidBody->setRestitution(COE);
  dynamicsWorld->addRigidBody(fallRigidBody);
  return fallRigidBody;
}

void bullet_init() {
  /*
   * set up world
   */
  broadphase = new btDbvtBroadphase();
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  solver = new btSequentialImpulseConstraintSolver;
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0., GRAVITY, 0));
  /*
   * Set up ground
   */
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  groundRigidBody->setRestitution(COE);
  dynamicsWorld->addRigidBody(groundRigidBody);
  /*
   * Set up left
   */
  btCollisionShape* leftShape = new btStaticPlaneShape(btVector3(1,0,0),1);
  btDefaultMotionState* leftMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(-50, 0,0)));
  btRigidBody::btRigidBodyConstructionInfo leftRigidBodyCI(0,leftMotionState,leftShape,btVector3(0,0,0));
  btRigidBody* leftRigidBody = new btRigidBody(leftRigidBodyCI);
  leftRigidBody->setRestitution(COE);
  dynamicsWorld->addRigidBody(leftRigidBody);
  /*
   * Set up right
   */
  btCollisionShape* rightShape = new btStaticPlaneShape(btVector3(-1,0,0),1);
  btDefaultMotionState* rightMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(50, 0,0)));
  btRigidBody::btRigidBodyConstructionInfo rightRigidBodyCI(0,rightMotionState,rightShape,btVector3(0,0,0));
  btRigidBody* rightRigidBody = new btRigidBody(rightRigidBodyCI);
  rightRigidBody->setRestitution(COE);
  dynamicsWorld->addRigidBody(rightRigidBody);
  /*
   * Set up top
   */
  btCollisionShape* topShape = new btStaticPlaneShape(btVector3(0,-1,0),1);
  btDefaultMotionState* topMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0, 100,0)));
  btRigidBody::btRigidBodyConstructionInfo topRigidBodyCI(0,topMotionState,topShape,btVector3(0,0,0));
  btRigidBody* topRigidBody = new btRigidBody(topRigidBodyCI);
  topRigidBody->setRestitution(COE);
  dynamicsWorld->addRigidBody(topRigidBody);
  /*
   * Set up front
   */
  btCollisionShape* frontShape = new btStaticPlaneShape(btVector3(0,0,1),1);
  btDefaultMotionState* frontMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0, 0, 0)));
  btRigidBody::btRigidBodyConstructionInfo frontRigidBodyCI(0,frontMotionState,frontShape,btVector3(0,0,0));
  btRigidBody* frontRigidBody = new btRigidBody(frontRigidBodyCI);
  frontRigidBody->setRestitution(COE);
  dynamicsWorld->addRigidBody(frontRigidBody);
  /*
   * Set up back
   */
  btCollisionShape* backShape = new btStaticPlaneShape(btVector3(0,0,-1),1);
  btDefaultMotionState* backMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0, 0, 100)));
  btRigidBody::btRigidBodyConstructionInfo backRigidBodyCI(0,backMotionState,backShape,btVector3(0,0,0));
  btRigidBody* backRigidBody = new btRigidBody(backRigidBodyCI);
  backRigidBody->setRestitution(COE);
  dynamicsWorld->addRigidBody(backRigidBody);
  /*
   * Set up sphere 0
   */
  MovingBits.push_back(SetSphere(5., btTransform(btQuaternion(0,0,1,1),btVector3(-10,45,0))));
  /*
   * Set up sphere 1
   */
  MovingBits.push_back(SetSphere(5., btTransform(btQuaternion(0,1,0,1),btVector3(-10,25,0))));
  /*
   * Set up sphere 2
   */
  MovingBits.push_back(SetSphere(5., btTransform(btQuaternion(1,0,0,1),btVector3(-10,65,0))));
  /*
   * Set up cone
   */
  MovingBits.push_back(SetCone(7.5, btTransform(btQuaternion(0,1,1,1),btVector3(-10,30,10))));

  Print("Setup Bullet ");
  int n = MovingBits.size();
  print(n);
}
glm::vec3 bullet_step(int i) {
  btTransform trans;
  btRigidBody* moveRigidBody;
  int n = MovingBits.size();
  moveRigidBody = MovingBits[i];
  dynamicsWorld->stepSimulation(1 / 60.f,10);
  moveRigidBody->getMotionState()->getWorldTransform(trans);
  return glm::vec3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}
void bullet_close() {
  /*
   * This is very minimal and relies on OS to tidy up.
   */
  btRigidBody* moveRigidBody;
  moveRigidBody = MovingBits[0];
  dynamicsWorld->removeRigidBody(moveRigidBody);
  delete moveRigidBody->getMotionState();
  delete moveRigidBody;
  delete dynamicsWorld;
  delete solver;
  delete collisionConfiguration;
  delete dispatcher;
  delete broadphase;
}
void Render(GLSLProgram O, VertexBufferObject vb) {
  glm::mat4 Projection = glm::perspective(45.0f, 1.0f, -10.0f, 10.0f);

  glm::vec3 eye = glm::vec3(0.0, 50.0, -105);
  glm::vec3 center = glm::vec3(0.0, 50.0, 0.0);
  glm::vec3 up = glm::vec3(0.0, 1.0, 0.0);
  glm::mat4 View = glm::lookAt(eye, center, up);

  glm::mat4 Model = glm::mat4(1.);

  O.Use();
  vb.SelectVAO();
  O.SetUniform("uProjection", Projection);
  O.SetUniform("uView", View);
  for(int i = 0; i < MovingBits.size() - 1; i++) {    // loop over shapes
    glm::vec3 position = bullet_step(i);
    Model = glm::translate(position);
    O.SetUniform("uModel", Model);
    vb.Draw();
  }
  vb.DeSelectVAO();
}


void RenderCone(GLSLProgram O, VertexBufferObject vb) {
  glm::mat4 Projection = glm::perspective(45.0f, 1.0f, -10.0f, 10.0f);

  glm::vec3 eye = glm::vec3(0.0, 50.0, -105);
  glm::vec3 center = glm::vec3(0.0, 50.0, 0.0);
  glm::vec3 up = glm::vec3(0.0, 1.0, 0.0);
  glm::mat4 View = glm::lookAt(eye, center, up);

  glm::mat4 Model = glm::mat4(1.);

  O.Use();
  vb.SelectVAO();
  O.SetUniform("uProjection", Projection);
  O.SetUniform("uView", View);
  glm::vec3 position = bullet_step(3);
  Model = glm::translate(position);
  O.SetUniform("uModel", Model);
  vb.Draw();
  vb.DeSelectVAO();
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  if ((key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) && action == GLFW_PRESS)
    glfwSetWindowShouldClose(window, GL_TRUE);
}
VertexBufferObject makeWireBoxMesh(void) {
  VertexBufferObject Box;
  Box.vboName = "Box";
  Box.SetVerbose(true);
  Box.CollapseCommonVertices( false );
  Box.SetTol( .001f );  // how close need to be to collapse vertices, ignored at the moment.
  Box.UseBufferObjects(true);   // Not needed as this is the only option.
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  Box.glBegin( GL_QUADS );

  // Front
  Box.glVertex3f(-50., 0., 0.);
  Box.glVertex3f(-50., 100., 0.);
  Box.glVertex3f(50., 100., 0.);
  Box.glVertex3f(50., 0., 0.);

  // LEFT
  Box.glVertex3f(-50., 0., 0.);
  Box.glVertex3f(-50., 0., 100.);
  Box.glVertex3f(-50., 100., 100.);
  Box.glVertex3f(-50., 100., 0.);

  // RIGHT
  Box.glVertex3f(50., 0., 0.);
  Box.glVertex3f(50., 0., 100.);
  Box.glVertex3f(50., 100., 100.);
  Box.glVertex3f(50., 100., 0.);

  // TOP
  Box.glVertex3f(-50., 100., 0.);
  Box.glVertex3f(-50., 100., 100.);
  Box.glVertex3f(50., 100., 100.);
  Box.glVertex3f(50., 100., 0.);

  // BOTTOM
  Box.glVertex3f(-50., 0., 0.);
  Box.glVertex3f(-50., 0., 100.);
  Box.glVertex3f(50., 0., 100.);
  Box.glVertex3f(50., 0., 0.);

  Box.glEnd();
  Box.Print();
  return Box;
}

VertexBufferObject makeWireCircleMesh(float radius) {
  VertexBufferObject Circle;
  Circle.vboName = "Circle";
  Circle.SetVerbose(true);
  Circle.CollapseCommonVertices( false );
  Circle.SetTol( .001f );  // how close need to be to collapse vertices, ignored at the moment.
  Circle.UseBufferObjects(true);   // Not needed as this is the only option.
  const float Radius = radius;
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  Circle.glBegin( GL_TRIANGLES );

  // Draw the vertices of a sphere instead of a circle
  int i;
  int numOfVertices = 4;
  Facet *f = NULL;
  f = (Facet *)malloc((int)pow(4,numOfVertices) * 8 * sizeof(Facet));
  numOfVertices = CreateUnitSphere(numOfVertices,f);
  for(i = 0; i<numOfVertices; i++){
    Circle.glVertex3f(f[i].p1.position[0]*Radius,f[i].p1.position[1]*Radius,f[i].p1.position[2]*Radius);
    Circle.glVertex3f(f[i].p2.position[0]*Radius,f[i].p2.position[1]*Radius,f[i].p2.position[2]*Radius);
    Circle.glVertex3f(f[i].p3.position[0]*Radius,f[i].p3.position[1]*Radius,f[i].p3.position[2]*Radius);
  }

  Circle.glEnd();
  Circle.Print();
  Circle.makeObj("Circle.obj");
  return Circle;
}

VertexBufferObject makeConeMesh(float radius) {
  VertexBufferObject Cone;
  Cone.vboName = "Cone";
  Cone.SetVerbose(true);
  Cone.CollapseCommonVertices( false );
  Cone.SetTol( .001f );
  Cone.UseBufferObjects(true);
  const float Radius = radius;
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  Cone.glBegin( GL_TRIANGLE_FAN );

  // Draw the vertices of a cone
  Vertex t;
  printf("Cone apex at: %f, %f, %f\n", (float)t.position[0], (float)t.position[1], (float)t.position[2]);
  Cone.glVertex3f(t.position[0], t.position[1], t.position[2]); // Apex
  int lod = 32;
  float step = 2. * 3.141596 / float(lod);
  for(float a = 0; a <= (2. * 3.141596 + step); a += step) {
    float c = Radius * cos(a);
    float s = Radius * sin(a);
    t.position[0] = c;
    t.position[1] = -15.0;
    t.position[2] = s; // set to 0.0 for a circle, >= 1.0 for a cone.
    Cone.glVertex3f(t.position[0], t.position[1], t.position[2]);
  }

  Cone.glEnd();
  Cone.Print();
  Cone.makeObj("Cone.obj");
  printf("Calculated cone\n");
  return Cone;
}

int main( void ) {
  int k = 0;
  bool good;
  GLFWwindow* window;
  if( !glfwInit() ) {
    printf("Failed to start GLFW\n");
    exit( EXIT_FAILURE );
  }
  window = glfwCreateWindow(500, 500, "Visible axies", NULL, NULL);// Initially 640, 480
  if (!window) {
    glfwTerminate();
    printf("GLFW Failed to start\n");
    return -1;
  }
  /* Make the window's context current */
  glfwMakeContextCurrent(window);   // IMPORTANT: Must be done so glew recognises OpenGL
  glewExperimental = GL_TRUE;
  int err = glewInit();
  if (GLEW_OK != err) {
    /* Problem: glewInit failed, something is seriously wrong. */
    fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
  }
  fprintf(stderr, "Glew done\n");
  glfwSetKeyCallback(window, key_callback);
  fprintf(stderr, "GL INFO %s\n", glGetString(GL_VERSION));
  /*
   * Vertex information
   */
  VertexBufferObject Circle = makeWireCircleMesh(7.5);
  VertexBufferObject Box  = makeWireBoxMesh();
  VertexBufferObject Cone = makeConeMesh(7.5);
  /*
   * shader programs for circle.
   */
  GLSLProgram O2;
  O2.SetVerbose(true);
  O2.SetGstap(false);
  good = O2.Create("box.vert", "box.frag");
  if( !good ) {
    fprintf( stderr, "GLSL Program wasn�t created.\n" );
    exit(0);
  }
  Circle.SelectVAO();
  O2.SetAttribute( "aPosition", Circle, VERTEX_ATTRIBUTE );   // or NORMAL_ATTRIBUTE or TEXTURE_ATTRIBUTE or ...
  Circle.DeSelectVAO();
  printf("Hello, set attributes for circle\n");
  /*
   * shader programs for boundary.
   */
  GLSLProgram O3;
  O3.SetVerbose(true);
  O3.SetGstap(false);
  good = O3.Create("box.vert", "box.frag");
  if( !good ) {
    fprintf( stderr, "GLSL Program wasn�t created.\n" );
    exit(0);
  }
  Box.SelectVAO();
  O3.SetAttribute( "aPosition", Box, VERTEX_ATTRIBUTE );   // or NORMAL_ATTRIBUTE or TEXTURE_ATTRIBUTE or ...
  Box.DeSelectVAO();

  GLSLProgram O4;
  O4.SetVerbose(true);
  O4.SetGstap(false);
  good = O4.Create("box.vert", "box.frag");
  if( !good ) {
    fprintf( stderr, "GLSL Program wasn�t created.\n" );
    exit(0);
  }
  Cone.SelectVAO();
  O4.SetAttribute( "aPosition", Cone, VERTEX_ATTRIBUTE );   // or NORMAL_ATTRIBUTE or TEXTURE_ATTRIBUTE or ...
  Cone.DeSelectVAO();

  bullet_init();   // set up physics

  glEnable(GL_DEPTH_TEST);
  Check("Before render loop");
  glClearColor(0.0, 0., 0., 1.0);  /* Make our background black */
  print("Starting to render");
  while(!glfwWindowShouldClose(window)) {   // Main loop
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 Projection = glm::perspective(45.0f, 1.0f, -10.0f, 10.0f);

    glm::vec3 eye = glm::vec3(0.0, 50.0, -105);// 0 50 -105
    glm::vec3 center = glm::vec3(0.0, 50.0, 0.0);
    glm::vec3 up = glm::vec3(0.0, 1.0, 0.0);
    glm::mat4 View = glm::lookAt(eye, center, up);

    glm::mat4 Model = glm::mat4(1.);
    O3.Use();
    O3.SetUniform("uProjection", Projection);
    O3.SetUniform("uView", View);
    O3.SetUniform("uModel", Model);
    Box.SelectVAO();
    Box.Draw();
    Box.DeSelectVAO();

    O4.Use();
    k++;
    RenderCone(O4, Cone);

    O2.Use();
    k++;
    Render(O2, Circle);

    glFlush();
    glfwSwapBuffers(window);    // Swap front and back rendering buffers
    glfwPollEvents();     // Poll for events.
  }
  void bullet_close();
  glfwTerminate();  // Close window and terminate GLFW
  exit( EXIT_SUCCESS );  // Exit program
}
