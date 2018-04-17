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
#define MSPF (1)
#define VIT (6)
#define PIT (8)
#define INCH (0.0254f)
#define MAX_FORCE (100000.0f)
#define GRAVITY (9.81f)

#define DIAMETER (0.001f)
#define RAD (DIAMETER/2.0f)

#define LENGTH 100
#define HEIGHT 80
#define WALL (1.0f/8.0f*INCH)

// parameters
//#define GAMMA (8.35f)
#define F (14) // 2 ~ 14

#define AMP (0.003f) // 0.002f ~ 0.006f
//#define FREQ sqrt(GAMMA*GRAVITY/(4.0f*M_PI*M_PI*AMP))
#define FREQ (15.0f) // <<- override GAMMA
#define GAMMA (4.0f*M_PI*M_PI*AMP*FREQ*FREQ/GRAVITY)
#define max(a,b) (((a)>(b))?(a):(b))

b2World* m_world;
b2PrismaticJoint* m_vjoint;
int32 mainWindow;
float32 current_time=0;
float32 dt = DT;
int32 mspf = 16;

bool pause=true;
bool draw=true;

void key(unsigned char k, int, int){
	switch(k){
		case 'd':
			draw=!draw;
			break;
		case 'p':
			pause=!pause;
			break;
		case 's':
			mspf *= 2;
			//slow
			//dt *= 0.5;
			break;
		case 'f':
			mspf = max(mspf/2,1);
			//mspf /= 2;
			//mspf = mspf<1?1:mspf; // min
			break;
		case 'q':
			glutDestroyWindow(mainWindow);
			exit(0);
		default:
			break;
	}
}

void loop(){
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);

	glViewport(0,0,w,h);
	float a = float(h) / w;
	float s = 0.12f;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	LoadOrtho2DMatrix(-s, s, -s*a, s*a);

	//for(int32 i=0; i<6000; ++i){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float omega = 2*M_PI*FREQ; // angular freq.

	if(!pause){
		m_vjoint->SetMotorSpeed(AMP * omega * sin(omega*current_time));
		std::cout << current_time << ',' << m_vjoint->GetJointTranslation() << std::endl;
		m_world->Step(dt, VIT, PIT);
		current_time += dt;
	}
	if(draw){
		m_world->DrawDebugData();
	}

	glutSwapBuffers();
}

static void Timer(int)
{
	glutSetWindow(mainWindow);
	glutPostRedisplay();
	glutTimerFunc(mspf, Timer, 0);
}

float randf(){
	return rand() / float(RAND_MAX);
}

b2PrismaticJoint* createContainer(
		float length,
		float height,
		float thickness,
		b2Body* groundBody,
		float groundthickness
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
	pd.groupFlags = b2_solidParticleGroup;// | b2_rigidParticleGroup;
	pd.flags = b2_staticPressureParticle;

	b2Vec2 pos[LENGTH*F];

	// give some clearance ... 	
	float clr = 1.0*DIAMETER; //2.0 spacing between particles
	int32 n = F*LENGTH;
	float32 x=clr, y=clr;
	for(int32 i=0; i<n; ++i){
		pos[i].Set(x,y);
		x += clr;
		if(x+clr > DIAMETER*LENGTH){
			x=clr;
			y+=clr;
		}
	}

	pd.particleCount = n;
	pd.positionData = pos;
	pd.angle = 0.0;
	pd.position.Set(-0.05, 0.0);
	pd.linearVelocity.SetZero();
	ps->CreateParticleGroup(pd);

	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f,-0.050f-WALL);
	b2PolygonShape groundBox;

	groundBox.SetAsBox(0.050f, WALL); //50cm, 10cm

	b2Body* groundBody = m_world->CreateBody(&groundBodyDef);
	groundBody->CreateFixture(&groundBox, 4000.0f);

	// container ..
	m_vjoint = createContainer(
			LENGTH*DIAMETER,
			HEIGHT*DIAMETER,
			WALL,
			groundBody, WALL);

	glutInit(&argc, argv);
	glutInitContextVersion(2, 0);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(640, 480);

	char title[32];
	sprintf(title, "VGM F=%d, f=%.2f, gamma=%.2f", F, FREQ, GAMMA);

	mainWindow = glutCreateWindow(title);
	glutSetWindow(mainWindow);
	glutDisplayFunc(loop);
	glutKeyboardFunc(key);

	
	uint32 flags = 0;
	flags |= b2Draw::e_shapeBit;
	flags |= b2Draw::e_particleBit;
	flags |= b2Draw::e_jointBit;
	//flags |= b2Draw::e_aabbBit;
	//flags |= b2Draw::e_centerOfMassBit;
	DebugDraw m_debugDraw;
	m_debugDraw.SetFlags(flags);
	m_world->SetDebugDraw(&m_debugDraw);
	
	glutTimerFunc(mspf, Timer, 0);
	glutMainLoop();

	delete m_world;
	return 0;
}
