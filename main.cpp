#include "Box2D/Common/b2Settings.h"

// override slop settings
#ifdef b2_linearSlop
#undef b2_linearSlop
#endif
#define b2_linearSlop 0.002f

#include "Box2D/Box2D.h"

#include "GL/freeglut.h"
#include "Render.h"
#include <iostream>

#define DT (1.0f/600.0f)
#define MSPF (16)
#define VIT (6)
#define PIT (8)
#define INCH (0.0254f)
#define MAX_FORCE (1000.0f)
#define GRAVITY (9.81f)

#define RAD 0.001f

#define LENGTH 100
#define HEIGHT 80
#define P_N LENGTH*F

// parameters
#define GAMMA (16.0f)
#define F (7) // 2 ~ 14

#define AMP (0.003f) // 0.002f ~ 0.006f
#define FREQ sqrt(GAMMA*GRAVITY/(4.0f*M_PI*M_PI*AMP))

//*AMP*M_PI*M_PI*FREQ*FREQ/GRAVITY

b2World* m_world;
b2PrismaticJoint* m_vjoint;
int32 mainWindow;
float32 current_time=0;

void loop(){
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	LoadOrtho2DMatrix(-0.2, 0.2, -0.2, 0.2);

	//for(int32 i=0; i<6000; ++i){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float omega = 2*M_PI*FREQ; // angular freq.

	m_vjoint->SetMotorSpeed(AMP * omega * sin(omega*current_time));

	m_world->Step(DT, VIT, PIT);
	current_time += DT;

	m_world->DrawDebugData();
	glutSwapBuffers();

}

static void Timer(int)
{
	glutSetWindow(mainWindow);
	glutPostRedisplay();
	glutTimerFunc(MSPF, Timer, 0);
}

float randf(){
	return rand() / float(RAND_MAX);
}

b2PrismaticJoint* createContainer(
		float length=0.100f,
		float height=0.080f,
		float thickness=(1.0/8.0)*INCH,
		b2Body* groundBody=nullptr,
		float groundthickness=0.0
		){
	if(!groundBody || groundthickness <= 0.0)
		return nullptr;

	float l2 = length/2.0;
	float t1 = thickness;
	float t2 = thickness/2.0;
	float h2 = height/2.0;

	b2BodyDef topDef, bottomDef, leftDef, rightDef;
	b2PolygonShape topBox, bottomBox, leftBox, rightBox;

	topDef.position.Set(0.0, h2+t2);
	bottomDef.position.Set(0.0, -h2-t2);
	leftDef.position.Set(-l2-t2, 0.0);
	rightDef.position.Set(l2+t2, 0.0);

	topDef.type = b2_dynamicBody;
	bottomDef.type = b2_dynamicBody;
	leftDef.type = b2_dynamicBody;
	rightDef.type = b2_dynamicBody;

	topBox.SetAsBox(l2+t1, t2); 
	bottomBox.SetAsBox(l2+t1, t2); 
	leftBox.SetAsBox(t2, h2);
	rightBox.SetAsBox(t2, h2);

	b2Body* topBody =m_world->CreateBody(&topDef);
	b2Body* bottomBody=m_world->CreateBody(&bottomDef);
	b2Body* leftBody =m_world->CreateBody(&leftDef);
	b2Body* rightBody=m_world->CreateBody(&rightDef);

	topBody->CreateFixture(&topBox, 1150.0f);
	bottomBody->CreateFixture(&bottomBox, 1150.0f);
	leftBody->CreateFixture(&leftBox, 1150.0f);
	rightBody->CreateFixture(&rightBox, 1150.0f);

	b2Vec2 tl(-l2-t2,h2), tr(l2+t2,h2);
	b2Vec2 bl(-l2-t2,-h2), br(l2+t2,-h2);

	b2WeldJointDef jtl, jtr;
	b2WeldJointDef jbl, jbr;
	
	jtl.bodyA = leftBody;
	jtl.bodyB = topBody;
	jtl.localAnchorA = leftBody->GetLocalPoint(tl);
	jtl.localAnchorB = topBody->GetLocalPoint(tl);
	jtl.collideConnected = false;

	jtr.bodyA = rightBody;
	jtr.bodyB = topBody;
	jtr.localAnchorA = rightBody->GetLocalPoint(tr);
	jtr.localAnchorB = topBody->GetLocalPoint(tr);
	jtr.collideConnected = false;

	jbl.bodyA = leftBody;
	jbl.bodyB = bottomBody;
	jbl.localAnchorA = leftBody->GetLocalPoint(bl);
	jbl.localAnchorB = bottomBody->GetLocalPoint(bl);
	jbl.collideConnected = false;

	jbr.bodyA = rightBody;
	jbr.bodyB = bottomBody;
	jbr.localAnchorA = rightBody->GetLocalPoint(br);
	jbr.localAnchorB = bottomBody->GetLocalPoint(br);
	jbr.collideConnected = false;


	m_world->CreateJoint(&jtl);
	m_world->CreateJoint(&jtr);
	m_world->CreateJoint(&jbl);
	m_world->CreateJoint(&jbr);

	// finally, vibration ...
	b2PrismaticJointDef jb;
	jb.localAxisA.Set(0.0f, 1.0f);
	jb.bodyA = groundBody;
	jb.bodyB = bottomBody;
	jb.collideConnected = false;

	jb.enableMotor = true;
	jb.maxMotorForce = MAX_FORCE;
	jb.motorSpeed = 0.0;

	jb.localAnchorA.Set(0.0, groundthickness); //5mm
	jb.localAnchorB.Set(0.0, -t2); 

	return (b2PrismaticJoint*)m_world->CreateJoint(&jb);
}

int main(int argc, char* argv[]){
	b2Vec2 gravity(0.0f, -9.81f);
	m_world = new b2World(gravity);

	// CREATE PARTICLE SYSTEM //
	b2ParticleSystemDef psd;
	auto ps = m_world->CreateParticleSystem(&psd);
	ps->SetRadius(RAD);
	ps->SetGravityScale(1.0f);
	ps->SetDensity(2000.0f);

	b2ParticleGroupDef pd;
	b2CircleShape shape; //1mm
	shape.m_radius = RAD;
	pd.shape = &shape;
	pd.particleCount = 100*F;
	pd.groupFlags = b2_solidParticleGroup;// | b2_rigidParticleGroup;

	b2Vec2 pos[100*F];
	//for(int32 i=0; i<P_N; ++i){
	//	pos[i].Set(0.04*randf()-0.02, 0.04*randf()-0.02);
	//	//pd.angle = (2 * M_PI) * randf() - M_PI;
	//	//pd.angularVelocity = randf();
	//	//ps->CreateParticleGroup(pd);
	//}
	
	for(int32 i=0; i<(F*2); ++i){
		for(int32 j=0; j<(100/2); ++j){
			int idx = i*(100/2)+j;
			if(idx < 100*F){
				pos[idx].Set(2*RAD*j+1*RAD, 2*RAD*i);
			}
		}
	}

	pd.positionData = pos;
	pd.angle = 0.0;
	pd.position.Set(-0.05, 0.0);
	pd.linearVelocity.SetZero();
	ps->CreateParticleGroup(pd);

	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f,-0.080f-0.002f);
	b2PolygonShape groundBox;

	groundBox.SetAsBox(0.050f, 0.002f); //50cm, 10cm

	b2Body* groundBody = m_world->CreateBody(&groundBodyDef);
	groundBody->CreateFixture(&groundBox, 4000.0f);

	// container ..
	m_vjoint = createContainer(
			LENGTH*RAD,
			HEIGHT*RAD,
			(1.0/8.0)*INCH,
			groundBody, 0.002f);

	glutInit(&argc, argv);
	glutInitContextVersion(2, 0);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);
	mainWindow = glutCreateWindow("title");
	glutSetWindow(mainWindow);
	glutDisplayFunc(loop);
	
	uint32 flags = 0;
	flags |= b2Draw::e_shapeBit;
	flags |= b2Draw::e_particleBit;
	flags |= b2Draw::e_jointBit;
	//flags |= b2Draw::e_aabbBit;
	//flags |= b2Draw::e_centerOfMassBit;
	DebugDraw m_debugDraw;
	m_debugDraw.SetFlags(flags);
	m_world->SetDebugDraw(&m_debugDraw);
	
	glutTimerFunc(MSPF, Timer, 0);
	glutMainLoop();

	delete m_world;
	return 0;
}
