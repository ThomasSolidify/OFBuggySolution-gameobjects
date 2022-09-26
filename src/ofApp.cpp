#include "ofApp.h"
#include "palletobject.h"

#define LENGTH 0.7      // chassis length
#define WIDTH 0.5       // chassis width
#define HEIGHT 0.2      // chassis height
#define RADIUS 0.18     // wheel radius
#define STARTZ 0.5      // starting height of chassis
#define CMASS 1         // chassis mass
#define WMASS 0.2       // wheel mass

static const dVector3 yunit = { 0, 1, 0 }, zunit = { 0, 0, 1 };

//--------------------------------------------------------------
void ofApp::setup(){

    /* Ensure texture sizes are normalized */
    /* Note that textures must be square! */
    ofDisableArbTex();

    int i;
    dMass m;
    speed=0, steer=0;

    // create world
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    dWorldSetGravity (world,0,0,-0.5);
    ground = dCreatePlane (space,0,0,1,0);

    // chassis body
    body[0] = dBodyCreate (world);
    dBodySetPosition (body[0],0,0,STARTZ);
    dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
    dMassAdjust (&m,CMASS);
    dBodySetMass (body[0],&m);
    box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
    dGeomSetBody (box[0],body[0]);

    // wheel bodies
    for (i=1; i<=3; i++) {
      body[i] = dBodyCreate (world);
      dQuaternion q;
      dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
      dBodySetQuaternion (body[i],q);
      dMassSetSphere (&m,1,RADIUS);
      dMassAdjust (&m,WMASS);
      dBodySetMass (body[i],&m);
      sphere[i-1] = dCreateSphere (0,RADIUS);
      dGeomSetBody (sphere[i-1],body[i]);
    }
    dBodySetPosition (body[1],0.5*LENGTH,0,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[2],-0.5*LENGTH, WIDTH*0.5,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[3],-0.5*LENGTH,-WIDTH*0.5,STARTZ-HEIGHT*0.5);

    // front and back wheel hinges
    for (i=0; i<3; i++) {
      joint[i] = dJointCreateHinge2 (world,0);
      dJointAttach (joint[i],body[0],body[i+1]);
      const dReal *a = dBodyGetPosition (body[i+1]);
      dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
      dJointSetHinge2Axes (joint[i], zunit, yunit);
    }
    // set joint suspension
    for (i=0; i<3; i++) {
      dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
      dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
    }

    // lock back wheels along the steering axis
    for (i=1; i<3; i++) {
      // set stops to make sure wheels always stay in alignment
      dJointSetHinge2Param (joint[i],dParamLoStop,0);
      dJointSetHinge2Param (joint[i],dParamHiStop,0);
      // the following alternative method is no good as the wheels may get out
      // of alignment:
      //   dJointSetHinge2Param (joint[i],dParamVel,0);
      //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
    }

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    dSpaceAdd (car_space,sphere[0]);
    dSpaceAdd (car_space,sphere[1]);
    dSpaceAdd (car_space,sphere[2]);

    // environment
    ground_box = dCreateBox (space,2,1.5,1);
    dMatrix3 R;
    dRFromAxisAndAngle (R,0,1,0,-0.15);
    dGeomSetPosition (ground_box,2,0,-0.34);
    dGeomSetRotation (ground_box,R);


    // Set up the OpenFrameworks camera
    ofVec3f upVector;
    upVector.set(0, 0, 1);
    cam.setAutoDistance(false);
    cam.setNearClip(0.01);
    cam.setPosition(10,10,10);
    cam.lookAt({0,0,0},upVector);
    cam.setUpAxis(upVector);

    dAllocateODEDataForThread(dAllocateMaskAll);

    /* Graphics ground plane */
    m_ground.set(8,8);              // 8x8 plane
    m_ground.mapTexCoords(0,0,4,4); // Texture tiles every 2 units
    m_ground.setResolution(128,128);    // How many triangles to divide the plane into.
                                    // Note that this affects lighting quality.

    /* The texture is saved in the bin/data directory.
     * It was found by searching "tiling dirt texture" ...
     */
    if(!ofLoadImage(m_groundTex, "dirt2.jpg")) { std::cerr << "Failed to load ground texture." << std::endl; }
    m_groundTex.setTextureWrap(GL_REPEAT, GL_REPEAT);

    /* The light */
    m_light1.setPosition(8,8,5);
    m_light1.lookAt(glm::vec3(0,0,0));
    m_light1.enable();

    /* Load the models */
    m_penguin.loadModel("penguin.dae", true);
    m_lowpolytree.loadModel("lowpolytree.dae", true);

    m_penguin.setRotation(0, 90.0, 1, 0, 0);
    m_lowpolytree.setRotation(0, 90.0, 1, 0, 0);

    std::cout << "Penguin normalized scale: " << m_penguin.getNormalizedScale() << std::endl;
    std::cout << "Penguin max: " << m_penguin.getSceneMax() << std::endl;
    std::cout << "Penguin min: " << m_penguin.getSceneMin() << std::endl;

    float scale;
    scale = 1.0 / m_penguin.getNormalizedScale();
    m_penguin.setScale(scale,scale,scale);

    std::cout << "Tree normalized scale: " << m_lowpolytree.getNormalizedScale() << std::endl;
    std::cout << "Tree max: " << m_lowpolytree.getSceneMax() << std::endl;
    std::cout << "Tree min: " << m_lowpolytree.getSceneMin() << std::endl;

    scale = 1.0 / m_lowpolytree.getNormalizedScale();
    m_lowpolytree.setScale(scale*50,scale*50,scale*50);

    /* Create some PalletObjects */
    for(unsigned int p=0; p<16; p++) {
        pallets.push_back(new PalletObject(ofRandom(-5,5), ofRandom(-5,5), ofRandom(0,10), world, space) );
    }
}

//--------------------------------------------------------------
void ofApp::update(){
    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

    // steering
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHinge2Param (joint[0],dParamVel,v);
    dJointSetHinge2Param (joint[0],dParamFMax,0.2);
    dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.05);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);

}
//--------------------------------------------------------------
void ofApp::draw(){

    // draw the scene
    ofBackground(20);
    cam.begin();

    ofEnableDepthTest();

    ofPushMatrix();

    ofSetColor(ofColor::lightGrey);
    //ofDrawGrid(0.2f,100, false, false,false,true);
    m_groundTex.bind();
    m_ground.draw();
    m_groundTex.unbind();

    ofDrawAxis(10);

    // chassis
    ofSetColor(ofColor::yellow);//FRONT
    const dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
    const dReal* pos_ode = dBodyGetPosition(body[0]);
    const dReal* rot_ode = dBodyGetQuaternion(body[0]);
    drawBox(pos_ode, rot_ode, sides);

    // wheels
    ofSetColor(ofColor::grey);
    for (int i=1; i<=3; i++) {
        drawCyl(dBodyGetPosition(body[i]),
                        dBodyGetQuaternion(body[i]),0.02f,RADIUS);
    }

    // ground box
    ofSetColor(ofColor::blue);
    dVector3 ss; dQuaternion r;
    dGeomBoxGetLengths (ground_box,ss);
    dGeomGetQuaternion(ground_box,r);
    drawBox(dGeomGetPosition(ground_box),r,ss);

    /* Draw a penguin at the origin */
    m_penguin.drawFaces();

    /* Draw some trees */
    for(int i=-4; i<=4; i+=4 )
        for(int j=-4; j<=4; j+=4 ) {
            if(i==j && i==0) continue;
            m_lowpolytree.setPosition(i,j,0);
            m_lowpolytree.drawFaces();
        }

    /* Draw the pallets */
    for(auto x: pallets ) x->draw();

    ofDisableDepthTest();
    cam.end();

    ofPopMatrix();
}
//--------------------------------------------------------------
void ofApp::exit() {
        dGeomDestroy (box[0]);
        dGeomDestroy (sphere[0]);
        dGeomDestroy (sphere[1]);
        dGeomDestroy (sphere[2]);
        dJointGroupDestroy (contactgroup);
        dSpaceDestroy (space);
        dWorldDestroy (world);
        dCloseODE();
}
//--------------------------------------------------------------
static void nearCallback (void *, dGeomID o1, dGeomID o2) {

    myApp->collide(o1,o2);
}

void ofApp::drawCyl(const dReal*pos_ode, const dQuaternion rot_ode, dReal len, dReal rad)
{
    ofCylinderPrimitive c;

    // ODE's and glm's quaternions are stored in a different order to openFrameworks:
    // ofQuaternion::ofQuaternion(float x, float y, float z, float w)
    // dQuaternion	dReal[4]	[ w, x, y, z ], where w is the real part and (x, y, z) form the vector part.
    // glm::quat(w,x,y,z)
    ofQuaternion rot_of(rot_ode[1], rot_ode[2], rot_ode[3], rot_ode[0]);

    // The OF Cylinder lies along the y axis (its length is along Y); ODE's stands tall in Z.
    // Let's fix that by creating a simple "rotate around X axis by 90 degrees" quaternion:
    ofQuaternion fix_cy; fix_cy.makeRotate(90,1,0,0);

    // And create a final orientation by combining that first rotation with the actual orientation of
    // the cylinder we got from ODE (in of quaternion ordering):
    ofQuaternion rot_final = fix_cy * rot_of;

    // ofCylinder dimensions: its length is 80 and is radius 60
    // std::cout << c.getHeight() << ", " <<c.getRadius() << std::endl;
    // scale it to be unit length and radius * the actual size:
    c.setScale(glm::vec3(rad/60.0,len/80.0,rad/60.0));

    // Use our calculated quaternion to orient the cylinder properly:
    c.setGlobalOrientation(rot_final);

    // Now set the cylinder's position according to ODE physics:
    c.setGlobalPosition(glm::vec3(pos_ode[0],pos_ode[1],pos_ode[2]));

    // Draw it:
    c.draw();
}

void ofApp::drawBox(const dReal*pos_ode, const dQuaternion rot_ode, const dReal*sides_ode)
{
    ofBoxPrimitive b;
    // ofBox dimensions: 100 * 100 * 100
    // std::cout << b.getSize() << std::endl;

    // scale it to be unit w, h, d * the actual size:
    b.setScale(glm::vec3(0.01*sides_ode[0],0.01*sides_ode[1],0.01*sides_ode[2]));

    // Simply set the orientation based on ODE's quaternion. Since we are using glm::quat
    // this time, the ordering is the same as ODE:
    b.setGlobalOrientation(glm::quat(rot_ode[0],rot_ode[1],rot_ode[2],rot_ode[3]));

    // Now set the box's position according to ODE physics:
    b.setGlobalPosition(glm::vec3(pos_ode[0],pos_ode[1],pos_ode[2]));

    // Draw it:
    b.draw();

}

void ofApp::collide(dGeomID o1, dGeomID o2)
{
  int i,n;

  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box);
  int g2 = (o2 == ground || o2 == ground_box);
  //if (!(g1 ^ g2)) return;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
        dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
                    dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
    }
  }
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){

    switch(key) {
    case 'a': case 'A':
      speed += 0.3;
      break;
    case 'z': case 'Z':
      speed -= 0.3;
      break;
    case ',':
      steer -= 0.5;
      break;
    case '.':
      steer += 0.5;
      break;
    case ' ':
      speed = 0;
      steer = 0;
        break;
    case 'q':
        ofExit();
        break;
    }
//    cout<<speed<<endl;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
