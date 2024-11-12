//|___________________________________________________________________
//!
//! \file plane2_base.cpp
//!
//! \brief Base source code for the second plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard inputs for plane and propeller (subpart):
//!   s   = moves the plane forward
//!   f   = moves the plane backward
//!   q,e = rolls the plane
//!   a   = yaws the plane
//!   x   = pitches the plane
//!
//!   r   = rotates propeller
//!
//! Mouse inputs for world-relative camera:
//!   Hold left button and drag  = controls azimuth and elevation 
//!                                (Press CTRL (and hold) before left button to restrict to azimuth control only,
//!                                 Press SHIFT (and hold) before left button to restrict to elevation control only)   
//!   Hold right button and drag = controls distance
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

//|___________________
//|
//| Includes
//|___________________

#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH        = 3;
const float P_LENGTH       = 3;
const float P_HEIGHT       = 1.5f;

// Plane transforms
const gmtl::Vec3f PLANE_FORWARD(0, 0, 1.0f);            // Plane's forward translation vector (w.r.t. local frame)
const float PLANE_ROTATION = 5.0f;                      // Plane rotated by 5 degs per input

// Propeller dimensions (subpart)
const float PP_WIDTH       = 3.0f;
const float PP_LENGTH      = 3.0f;

// Propeller transforms
const gmtl::Point3f STABILIZER_POS(P_WIDTH/2, 0, 0);     // Propeller position on the plane (w.r.t. plane's frame)
const float STABILIZER_ROTATION = 5.0f;                  // Propeller rotated by 5 degs per input

// Camera's view frustum 
const float CAM_FOV        = 90.0f;                     // Field of view in degs

// Keyboard modifiers
enum KeyModifier {KM_SHIFT = 0, KM_CTRL, KM_ALT};

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width    = 800;
int w_height   = 600;

// Plane pose (position-quaternion pair)
gmtl::Point4f plane_p1;      // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf   plane_q1;      // Quaternion

gmtl::Point4f plane_p2;     
gmtl::Quatf   plane_q2;

// Quaternions to rotate plane
gmtl::Quatf zrotp_q;        // Positive and negative Z rotations
gmtl::Quatf zrotn_q;

gmtl::Quatf xrotp_q;
gmtl::Quatf xrotn_q;

gmtl::Quatf yrotp_q;
gmtl::Quatf yrotn_q;

// Propeller rotation (subpart)
float tr_angle_a = 90;         
float tr_angle_a_sub = -90;
float st_angle_b = 90;
float st_angle_c = 90;

float pp_angle_limit = 35.0f;

float sub_b_max_angle = st_angle_b + pp_angle_limit;
float sub_b_min_angle = st_angle_b - pp_angle_limit;

float sub_c_max_angle = st_angle_c + pp_angle_limit;
float sub_c_min_angle = st_angle_c - pp_angle_limit;
 
// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3]   = {false, false, false};
bool kmodifiers[3] = {false, false, false};

// Cameras
int cam_id         = 0;                                // Selects which camera to view
int camctrl_id     = 0;                                // Selects which camera to control
float distance[3]  = { 20.0f,  20.0f,  20.0f};                 // Distance of the camera from world's origin.
float elevation[3] = {-45.0f, -45.0f, -45.0f};                 // Elevation of the camera. (in degs)
float azimuth[3]   = { 15.0f,  15.0f, -15.0f};                 // Azimuth of the camera. (in degs)

//|___________________
//|
//| Function Prototypes
//|___________________

void InitTransforms();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawPlaneBody(const float width, const float length, const float height);
void DrawStabilizer(const float width, const float length);
void DrawTurret(const float width, const float length, const float height);
void DrawTurretGun(const float width, const float length, const float height);

//|____________________________________________________________________
//|
//| Function: InitTransforms
//|
//! \param None.
//! \return None.
//!
//! Initializes all the transforms
//|____________________________________________________________________

void InitTransforms()
{
  const float COSTHETA_D2  = cos(gmtl::Math::deg2Rad(PLANE_ROTATION/2));  // cos() and sin() expect radians 
  const float SINTHETA_D2  = sin(gmtl::Math::deg2Rad(PLANE_ROTATION/2));

  // Inits plane pose
  plane_p1.set(1.0f, 0.0f, 4.0f, 1.0f);
  plane_q1.set(0, 0, 0, 1);

  // Z rotations (roll)
  zrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
  zrotn_q = gmtl::makeConj(zrotp_q);                // -Z

  // X rotation (pitch)
  xrotp_q.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X
  xrotn_q = gmtl::makeConj(xrotp_q);                // -x

  // Y rotation (yaw)
  yrotp_q.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
  yrotn_q = gmtl::makeConj(yrotp_q);                // -Y
}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
  glClearColor(0.7f, 0.7f, 0.7f, 1.0f); 
  glEnable(GL_DEPTH_TEST); 
  glShadeModel(GL_SMOOTH);
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
    gmtl::AxisAnglef aa;    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
    gmtl::Vec3f axis;       // Axis component of axis-angle representation
    float angle;            // Angle component of axis-angle representation

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(CAM_FOV, (float)w_width/w_height, 0.1f, 1000.0f);     // Check MSDN: google "gluPerspective msdn"

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

//|____________________________________________________________________
//|
//| Setting up view transform by:
//| "move up to the world frame by composing all of the (inverse) transforms from the camera up to the world node"
//|____________________________________________________________________

    switch (cam_id) {
    case 0:
        // For the world-relative camera
        glTranslatef(0, 0, -distance[0]);
        glRotatef(-elevation[0], 1, 0, 0);
        glRotatef(-azimuth[0], 0, 1, 0);
    break;

    case 1:
        // For plane2's camera
        glTranslatef(0, 0, -distance[1]);
        glRotatef(-elevation[1], 1, 0, 0);
        glRotatef(-azimuth[1], 0, 1, 0);

        gmtl::set(aa, plane_q1);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
        axis  = aa.getAxis();
        angle = aa.getAngle();
        glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
        glTranslatef(-plane_p1[0], -plane_p1[1], -plane_p1[2]);      
    break;

    case 2:
        glTranslatef(0, 0, -distance[2]);
        glRotatef(-elevation[2], 1, 0, 0);
        glRotatef(-azimuth[2], 0, 1, 0);

        gmtl::set(aa, plane_q1);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
        axis = aa.getAxis();
        angle = aa.getAngle();
        glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
        glTranslatef(-plane_p1[0], -plane_p1[1], -plane_p1[2]);
    break;
  }

//|____________________________________________________________________
//|
//| Draw traversal begins, start from world (root) node
//|____________________________________________________________________

  // World node: draws world coordinate frame
    DrawCoordinateFrame(10);
  // World-relative camera:
    if (cam_id != 0) {
        glPushMatrix();        
        glRotatef(azimuth[0], 0, 1, 0);
        glRotatef(elevation[0], 1, 0, 0);
        glTranslatef(0, 0, distance[0]);
        DrawCoordinateFrame(1);
        glPopMatrix();
    }
    // Plane 1 body:
    glPushMatrix();
    gmtl::set(aa, plane_q1);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
    axis  = aa.getAxis();
    angle = aa.getAngle();
    glTranslatef(plane_p1[0] + 5, plane_p1[1], plane_p1[2] + 5);
    glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
    DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
    DrawCoordinateFrame(3);
    // Plane 1's camera:
    glPushMatrix();
    glRotatef(azimuth[1], 0, 1, 0);
    glRotatef(elevation[1], 1, 0, 0);
    glTranslatef(0, 0, distance[1]);
    DrawCoordinateFrame(1);
    glPopMatrix();
    // Subpart a
    const float sub_a_x_offset = -1.5f;
    const float sub_a_y_offset = -1.3f;
    const float sub_a_z_offset = 0.0f;
    const float sub_sub_a_y_offset = -3.5f;
    glPushMatrix();
    glTranslatef(STABILIZER_POS[0] + sub_a_x_offset, STABILIZER_POS[1] + sub_a_y_offset, STABILIZER_POS[2] + sub_a_z_offset);
    glRotatef(tr_angle_a, 0, 1, 0);                                         
    DrawTurret(P_WIDTH, P_LENGTH, P_HEIGHT);
    DrawCoordinateFrame(1);
    // Sub-subpart
    glPushMatrix();
    glTranslatef(0, PP_LENGTH + sub_sub_a_y_offset, 0);
    glRotatef(tr_angle_a_sub, 0, 1, 0);
    DrawTurretGun(P_WIDTH, P_LENGTH, P_HEIGHT);
    DrawCoordinateFrame(1);
    glPopMatrix();
    glPopMatrix();
    // Subpart b
    const float sub_b_x_offset = 1.3f;
    const float sub_b_y_offset = -0.9f;
    const float sub_b_z_offset = -2.3f;
    glPushMatrix();   
    glTranslatef(STABILIZER_POS[0] + sub_b_x_offset, STABILIZER_POS[1] + sub_b_y_offset, STABILIZER_POS[2] + sub_b_z_offset);
    glRotatef(st_angle_b, 1, 0, 0);                                         
    DrawStabilizer(PP_WIDTH, PP_LENGTH);
    DrawCoordinateFrame(1);
    glPopMatrix();
    // Subpart c
    const float sub_c_x_offset = -4.3f;
    const float sub_c_y_offset = -0.9f;
    const float sub_c_z_offset = -2.3f;
    glPushMatrix();
    glTranslatef(STABILIZER_POS[0] + sub_c_x_offset, STABILIZER_POS[1] + sub_c_y_offset, STABILIZER_POS[2] + sub_c_z_offset);
    glRotatef(st_angle_c, 1, 0, 0);                                          
    DrawStabilizer(PP_WIDTH, PP_LENGTH);
    DrawCoordinateFrame(1);
    glPopMatrix();
    // End Plane 1
    glPopMatrix();
    // Plane 2 body:
    glPushMatrix();
    gmtl::set(aa, plane_q2);
    axis = aa.getAxis();
    angle = aa.getAngle();
    glTranslatef(plane_p2[0], plane_p2[1], plane_p2[2]);
    glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
    DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
    DrawCoordinateFrame(3);
    glPopMatrix();
    // Plane 2's camera:
    glPushMatrix();
    glRotatef(azimuth[2], 0, 1, 0);
    glRotatef(elevation[2], 1, 0, 0);
    glTranslatef(0, 0, distance[2]);
    DrawCoordinateFrame(1);
    glPopMatrix();
    // End Plane 2
    glPopMatrix();  
    glutSwapBuffers();                          // Replaces glFlush() to use double buffering
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param key    [in] Key code.
//! \param x      [in] X-coordinate of mouse when key is pressed.
//! \param y      [in] Y-coordinate of mouse when key is pressed.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
  switch (key) {
//|____________________________________________________________________
//|
//| Camera switch
//|____________________________________________________________________

    case 'v': // Select camera to view
      cam_id = (cam_id + 1) % 3;
      printf("View camera = %d\n", cam_id);
      break;
    case 'b': // Select camera to control
      camctrl_id = (camctrl_id + 1) % 3;
      printf("Control camera = %d\n", camctrl_id);
      break;

//|____________________________________________________________________
//|
//| Plane controls
//|____________________________________________________________________

    case 'p': { // Forward translation of the plane (+Z translation)  
      gmtl::Quatf v_q = plane_q1 * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q1);
      plane_p1         = plane_p1 + v_q.mData;
      } break;
    case ';': { // Backward translation of the plane (-Z translation)
      gmtl::Quatf v_q = plane_q1 * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q1);
      plane_p1         = plane_p1 + v_q.mData;
      } break;

    case 'e': // Rolls the plane (+Z rot)
      plane_q1 = plane_q1 * zrotp_q;
      break;
    case 'q': // Rolls the plane (-Z rot)
      plane_q1 = plane_q1 * zrotn_q;
      break;

    case 'w': // Pitches the plane (+X rot)
        plane_q1 = plane_q1 * xrotp_q;
        break;
    case 's': // Pitches the plane (+X rot)
        plane_q1 = plane_q1 * xrotn_q;
        break;
    case 'a': // Pitches the plane (+Y rot)
        plane_q1 = plane_q1 * yrotp_q;
        break;
    case 'd': // Rolls the plane (-Y rot)
        plane_q1 = plane_q1 * yrotn_q;
        break;

//|____________________________________________________________________
//|
//| Propeller controls (subpart)
//|____________________________________________________________________

    case 'r': // Rotates propeller
      st_angle_b += STABILIZER_ROTATION;
      if (st_angle_b > sub_b_max_angle) {
          st_angle_b = sub_b_max_angle;
      }

      break;
    case 'f': // Rotates propeller 
        st_angle_b -= STABILIZER_ROTATION;
        if (st_angle_b < sub_b_min_angle) {
            st_angle_b = sub_b_min_angle;
        }
        break;

    case 't': // Rotates propeller 
        st_angle_c += STABILIZER_ROTATION;
        if (st_angle_c > sub_c_max_angle) {
            st_angle_c = sub_c_max_angle;
        }
        break;
    case 'g': // Rotates propeller 
        st_angle_c -= STABILIZER_ROTATION;
        if (st_angle_c < sub_c_min_angle) {
            st_angle_c = sub_c_min_angle;
        }
        break;

    case 'y': // Rotates propeller 
        tr_angle_a += STABILIZER_ROTATION;
    
        break;
    case 'h': // Rotates propeller 
        tr_angle_a -= STABILIZER_ROTATION;
     
        break;

    case 'u': // Rotates propeller 
        tr_angle_a_sub += STABILIZER_ROTATION;

        break;
    case 'j': // Rotates propeller 
        tr_angle_a_sub -= STABILIZER_ROTATION;

        break;
  }

  glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MouseFunc
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void MouseFunc(int button, int state, int x, int y)
{
  int km_state;

  // Updates button's sate and mouse coordinates
  if (state == GLUT_DOWN) {
    mbuttons[button] = true;
    mx_prev          = x;
    my_prev          = y;
  } else {
    mbuttons[button] = false;
  }

  // Updates keyboard modifiers
  km_state = glutGetModifiers();
  kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
  kmodifiers[KM_CTRL]  = km_state & GLUT_ACTIVE_CTRL  ? true : false;
  kmodifiers[KM_ALT]   = km_state & GLUT_ACTIVE_ALT   ? true : false;

  //glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MotionFunc
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void MotionFunc(int x, int y)
{
  int dx, dy, d;

  if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON]) {
    // Computes distances the mouse has moved
    dx      = x - mx_prev;
    dy      = y - my_prev;

    // Updates mouse coordinates
    mx_prev = x;
    my_prev = y;

    // Hold left button to rotate camera
    if (mbuttons[GLUT_LEFT_BUTTON]) {
      if (!kmodifiers[KM_CTRL]) {        
        elevation[camctrl_id] += dy;            // Elevation update
      }
      if (!kmodifiers[KM_SHIFT]) {      
        azimuth[camctrl_id] += dx;             // Azimuth update
      }
    }

    // Hold right button to zoom
    if (mbuttons[GLUT_RIGHT_BUTTON]) {
      if (abs(dx) >= abs(dy)) {
        d = dx;
      } else {
        d = -dy;
      }
      distance[camctrl_id] += d;    
    }

    glutPostRedisplay();      // Asks GLUT to redraw the screen
  }
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
  // Track the current window dimensions
  w_width  = w;
  w_height = h;
  glViewport(0, 0, (GLsizei) w_width, (GLsizei) w_height);
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
  glBegin(GL_LINES);
    // X axis is red
    glColor3f( 1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(   l, 0.0f, 0.0f);

    // Y axis is green
    glColor3f( 0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(0.0f,    l, 0.0f);

    // Z axis is blue
    glColor3f( 0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
	  glVertex3f(0.0f, 0.0f,    l);
  glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawPlaneBody
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws a plane body.
//|____________________________________________________________________

void DrawPlaneBody(const float width, const float length, const float height)
{
    float w = width / 2;
    float l = length / 2;
    float h = height / 2;
    float lima = l * 1.5;
    float whiskey = w * 3;
    float tango = w - (w - 0.5);
    float hotel = h * 2.5;
    float tail_lima = l * 0.2;
    float hulu = h / 5;

    // Body
    glBegin(GL_QUADS);
    glColor3f(1.0f, 0.0f, 0.0f);
    // Front face
    glVertex3f(-w, -h, lima);
    glVertex3f(w, -h, lima);
    glVertex3f(w, h, lima);
    glVertex3f(-w, h, lima);
    // Back face
    glVertex3f(-w, -h, -lima);
    glVertex3f(-w, h, -lima);
    glVertex3f(w, -h, -lima);
    glVertex3f(w, h, -lima);
    // Right face
    glVertex3f(w, -h, -lima);
    glVertex3f(w, h, -lima);
    glVertex3f(w, h, lima);
    glVertex3f(w, -h, lima);
    // Left face
    glVertex3f(-w, -h, -lima);
    glVertex3f(-w, -h, lima);
    glVertex3f(-w, h, lima);
    glVertex3f(-w, h, -lima);
    // Top face
    glVertex3f(-w, h, -lima);
    glVertex3f(-w, h, lima);
    glVertex3f(w, h, lima);
    glVertex3f(w, h, -lima);
    // Bottom face
    glVertex3f(-w, -h, -lima);
    glVertex3f(w, -h, -lima);
    glVertex3f(w, -h, lima);
    glVertex3f(-w, -h, lima);
    // Bottom face close gap
    glVertex3f(-w, -h - 0.3, -lima);
    glVertex3f(w, -h - 0.3, -lima);
    glVertex3f(w, -h - 0.3, lima);
    glVertex3f(-w, -h - 0.3, lima);
    glEnd();

    // Cockpit
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 0.0f, 1.0f);
    // Right side
    glVertex3f(w, h, lima);
    glVertex3f(w, -h, lima);
    glVertex3f(w, -h, lima + l);
    // Left side
    glVertex3f(-w, h, lima);
    glVertex3f(-w, -h, lima);
    glVertex3f(-w, -h, lima + l);
    glEnd();

    glBegin(GL_QUADS);
    glColor3f(0.0f, 0.0f, 1.0f);
    // Front
    glVertex3f(-w, -h, lima + l);
    glVertex3f(-w, h, lima);
    glVertex3f(w, h, lima);
    glVertex3f(w, -h, lima + l);
    // Bottom
    glVertex3f(-w, -h, lima + l);
    glVertex3f(-w, -h, lima);
    glVertex3f(w, -h, lima);
    glVertex3f(w, -h, lima + l);
    // Bottom close gap
    glVertex3f(-w, -h - 0.3, lima + l);
    glVertex3f(-w, -h - 0.3, lima);
    glVertex3f(w, -h - 0.3, lima);
    glVertex3f(w, -h - 0.3, lima + l);
    glEnd();

    // Wings 
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 0.0f);
    // Right wing (top)
    glVertex3f(w, -h, -lima);
    glVertex3f(whiskey, -h, -lima);
    glVertex3f(w, -h, lima + l);

    // Right wing (bottom)
    glVertex3f(w, -h - 0.3, -lima);
    glVertex3f(whiskey, -h - 0.3, -lima);
    glVertex3f(w, -h - 0.3, lima + l);
    // Left wing (top)
    glVertex3f(-w, -h, -lima);
    glVertex3f(-whiskey, -h, -lima);
    glVertex3f(-w, -h, lima + l);
    // Left wing (bottom)
    glVertex3f(-w, -h - 0.3, -lima);
    glVertex3f(-whiskey, -h - 0.3, -lima);
    glVertex3f(-w, -h - 0.3, lima + l);
    glEnd();

    // Tail
    glBegin(GL_TRIANGLES);
    glColor3f(0.0f, 1.0f, 1.0f);
    // Right 
    glVertex3f(tango, h, -lima);
    glVertex3f(tango, hotel, -lima);
    glVertex3f(tango, h, tail_lima);
    // Left
    glVertex3f(-tango, h, -lima);
    glVertex3f(-tango, hotel, -lima);
    glVertex3f(-tango, h, tail_lima);
    glEnd();

    glBegin(GL_QUADS);
    glColor3f(0.0f, 1.0f, 1.0f);
    // Tail
    glVertex3f(-tango, h, tail_lima);
    glVertex3f(-tango, hotel, -lima);
    glVertex3f(tango, hotel, -lima);
    glVertex3f(tango, h, tail_lima);

    glVertex3f(-tango, h, -lima);
    glVertex3f(-tango, hotel, -lima);
    glVertex3f(tango, hotel, -lima);
    glVertex3f(tango, h, -lima);

    glEnd();

    ////close gap
    glBegin(GL_QUADS);
    glColor3f(0.0f, 0.0f, 0.0f);

    glVertex3f(-w, -h - 0.3, lima + l);
    glVertex3f(-w, -h, lima + l);
    glVertex3f(w, -h, lima + l);
    glVertex3f(w, -h - 0.3, lima + l);

    glVertex3f(w, -h - 0.3, lima + l);
    glVertex3f(w, -h, lima + l);
    glVertex3f(whiskey, -h, -lima);
    glVertex3f(whiskey, -h - 0.3, -lima);

    glVertex3f(-w, -h - 0.3, lima + l);
    glVertex3f(-w, -h, lima + l);
    glVertex3f(-whiskey, -h, -lima);
    glVertex3f(-whiskey, -h - 0.3, -lima);

    glVertex3f(whiskey, -h - 0.3, -lima);
    glVertex3f(whiskey, -h, -lima);
    glVertex3f(-whiskey, -h, -lima);
    glVertex3f(-whiskey, -h - 0.3, -lima);

    glEnd();
}

void DrawTurretGun(const float width, const float length, const float height) {
    float w = width / 2;
    float l = length / 2;
    float h = height / 2;

    // Gun specific dimensions
    float gun_barrel_length = l * 2.5f; // Length of the gun barrel
    float gun_barrel_radius = w * 0.15f; // Radius of the gun barrel (width of the barrel)

    // Smaller head dimensions
    float gun_head_radius = w * 0.5f; // Smaller radius for the turret head
    float gun_head_height = h * 0.3f;  // Smaller height for the turret head

    // Position offsets for the head and barrel
    float head_offset = l * 0.5f;  // Offset for the center of the turret's head
    float barrel_offset = gun_barrel_length / 2.0f; // Offset for the barrel

    // Draw the turret head (a cylinder representing the turret's rotating part)
    glBegin(GL_QUADS);
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow color
    // Top face
    glVertex3f(-gun_head_radius, gun_head_height, head_offset);
    glVertex3f(gun_head_radius, gun_head_height, head_offset);
    glVertex3f(gun_head_radius, gun_head_height, head_offset - l);
    glVertex3f(-gun_head_radius, gun_head_height, head_offset - l);

    // Bottom face
    glVertex3f(-gun_head_radius, -gun_head_height, head_offset);
    glVertex3f(-gun_head_radius, -gun_head_height, head_offset - l);
    glVertex3f(gun_head_radius, -gun_head_height, head_offset - l);
    glVertex3f(gun_head_radius, -gun_head_height, head_offset);

    // Right face
    glVertex3f(gun_head_radius, gun_head_height, head_offset);
    glVertex3f(gun_head_radius, -gun_head_height, head_offset);
    glVertex3f(gun_head_radius, -gun_head_height, head_offset - l);
    glVertex3f(gun_head_radius, gun_head_height, head_offset - l);

    // Left face
    glVertex3f(-gun_head_radius, gun_head_height, head_offset);
    glVertex3f(-gun_head_radius, gun_head_height, head_offset - l);
    glVertex3f(-gun_head_radius, -gun_head_height, head_offset - l);
    glVertex3f(-gun_head_radius, -gun_head_height, head_offset);

    // Front face (the side facing the gun barrel)
    glVertex3f(-gun_head_radius, gun_head_height, head_offset);
    glVertex3f(gun_head_radius, gun_head_height, head_offset);
    glVertex3f(gun_head_radius, -gun_head_height, head_offset);
    glVertex3f(-gun_head_radius, -gun_head_height, head_offset);

    // Back face
    glVertex3f(-gun_head_radius, gun_head_height, head_offset - l);
    glVertex3f(-gun_head_radius, -gun_head_height, head_offset - l);
    glVertex3f(gun_head_radius, -gun_head_height, head_offset - l);
    glVertex3f(gun_head_radius, gun_head_height, head_offset - l);
    glEnd();

    // Draw the gun barrel (a long cylinder extending forward from the head)
    glBegin(GL_QUADS);
    glColor3f(0.5f, 0.5f, 0.5f); // Barrel color (metallic gray)

    // Front face of the barrel
    glVertex3f(-gun_barrel_radius, gun_head_height, head_offset + barrel_offset);
    glVertex3f(gun_barrel_radius, gun_head_height, head_offset + barrel_offset);
    glVertex3f(gun_barrel_radius, -gun_head_height, head_offset + barrel_offset);
    glVertex3f(-gun_barrel_radius, -gun_head_height, head_offset + barrel_offset);

    // Back face of the barrel (attached to the turret head)
    glVertex3f(-gun_barrel_radius, gun_head_height, head_offset);
    glVertex3f(-gun_barrel_radius, -gun_head_height, head_offset);
    glVertex3f(gun_barrel_radius, -gun_head_height, head_offset);
    glVertex3f(gun_barrel_radius, gun_head_height, head_offset);

    // Right face of the barrel
    glVertex3f(gun_barrel_radius, gun_head_height, head_offset + barrel_offset);
    glVertex3f(gun_barrel_radius, -gun_head_height, head_offset + barrel_offset);
    glVertex3f(gun_barrel_radius, -gun_head_height, head_offset);
    glVertex3f(gun_barrel_radius, gun_head_height, head_offset);

    // Left face of the barrel
    glVertex3f(-gun_barrel_radius, gun_head_height, head_offset + barrel_offset);
    glVertex3f(-gun_barrel_radius, gun_head_height, head_offset);
    glVertex3f(-gun_barrel_radius, -gun_head_height, head_offset);
    glVertex3f(-gun_barrel_radius, -gun_head_height, head_offset + barrel_offset);

    // Top face of the barrel
    glVertex3f(-gun_barrel_radius, gun_head_height, head_offset + barrel_offset);
    glVertex3f(gun_barrel_radius, gun_head_height, head_offset + barrel_offset);
    glVertex3f(gun_barrel_radius, gun_head_height, head_offset);
    glVertex3f(-gun_barrel_radius, gun_head_height, head_offset);

    // Bottom face of the barrel
    glVertex3f(-gun_barrel_radius, -gun_head_height, head_offset + barrel_offset);
    glVertex3f(-gun_barrel_radius, -gun_head_height, head_offset);
    glVertex3f(gun_barrel_radius, -gun_head_height, head_offset);
    glVertex3f(gun_barrel_radius, -gun_head_height, head_offset + barrel_offset);
    glEnd();
}

void DrawTurret(const float width, const float length, const float height) {
    float w = width / 2;
    float l = length / 2;
    float h = height / 2;

    float w_offset = w - 0.5;
    float l_offset = l - 0.5;
    float h_offset = h - 0.5;

    float lima = l * 1.5;
    float whiskey = w * 3;
    float tango = w - (w - 0.5);
    float hotel = h * 2.5;
    float tail_lima = l * 0.2;
    float hulu = h / 5;

    // Body
    glBegin(GL_QUADS);
    glColor3f(0.6f, 0.3f, 0.1f);
    // Front face
    glVertex3f(-w_offset, -h_offset, l_offset);
    glVertex3f(w_offset, -h_offset, l_offset);
    glVertex3f(w_offset, h_offset, l_offset);
    glVertex3f(-w_offset, h_offset, l_offset);
    // Back face
    glVertex3f(-w_offset, -h_offset, -l_offset);
    glVertex3f(-w_offset, h_offset, -l_offset);
    glVertex3f(w_offset, -h_offset, -l_offset);
    glVertex3f(w_offset, h_offset, -l_offset);
    // Right face
    glVertex3f(w_offset, -h_offset, -l_offset);
    glVertex3f(w_offset, h_offset, -l_offset);
    glVertex3f(w_offset, h_offset, l_offset);
    glVertex3f(w_offset, -h_offset, l_offset);
    // Left face
    glVertex3f(-w_offset, -h_offset, -l_offset);
    glVertex3f(-w_offset, -h_offset, l_offset);
    glVertex3f(-w_offset, h_offset, l_offset);
    glVertex3f(-w_offset, h_offset, -l_offset);
    // Top face
    glVertex3f(-w_offset, h_offset, -l_offset);
    glVertex3f(-w_offset, h_offset, l_offset);
    glVertex3f(w_offset, h_offset, l_offset);
    glVertex3f(w_offset, h_offset, -l_offset);
    // Bottom face
    glVertex3f(-w_offset, -h_offset, -l_offset);
    glVertex3f(w_offset, -h_offset, -l_offset);
    glVertex3f(w_offset, -h_offset, l_offset);
    glVertex3f(-w_offset, -h_offset, l_offset);
    
    glEnd();
}

//|____________________________________________________________________
//|
//| Function: DrawStabilizer
//|
//! \param width       [in] Width  of the propeller.
//! \param length      [in] Length of the propeller.
//! \return None.
//!
//! Draws a propeller.
//|____________________________________________________________________

void DrawStabilizer(const float width, const float length)
{
  float w = width/2;
  float l = length/2;
  float lima = l / 6;
  float whiskey = w * 3;
  float tango = w - (w - 0.5);
  float tail_lima = l * 0.2;

    glBegin(GL_QUADS);

    // Front face
    glColor3f(1.0f, 0.5f, 0.0f);

    glVertex3f(-w, -lima, 0);
    glVertex3f(w, -lima, 0);
    glVertex3f(w, lima, 0);
    glVertex3f(-w, lima, 0);

    glEnd();
}

//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char **argv)
{ 
  InitTransforms();

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);     // Uses GLUT_DOUBLE to enable double buffering
  glutInitWindowSize(w_width, w_height);
  
  glutCreateWindow("Plane Episode 2");

  glutDisplayFunc(DisplayFunc);
  glutKeyboardFunc(KeyboardFunc);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MotionFunc);
  glutReshapeFunc(ReshapeFunc);
  
  InitGL();

  glutMainLoop();

  return 0;
}