//|___________________________________________________________________
//!
//! \file plane1_base.cpp
//!
//! \brief Base source code for the first plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Keyboard controls:
//!   s   = moves the plane forward
//!   f   = moves the plane backward
//!   q,e = rolls the plane
//!
//!   k   = moves the camera forward
//!   ;   = moves the camera backward
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
const float P_WIDTH  = 3;
const float P_LENGTH = 3;
const float P_HEIGHT = 1.5;

// Camera's view frustum 
const float CAM_FOV  = 60.0f;     // Field of view in degs

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width    = 800;
int w_height   = 600;

// Plane pose (position & orientation)
gmtl::Matrix44f plane_pose; // T, as defined in the handout, initialized to IDENTITY by default

// Camera pose
gmtl::Matrix44f cam_pose;   // C, as defined in the handout
gmtl::Matrix44f view_mat;   // View transform is C^-1 (inverse of the camera transform C)

gmtl::Matrix44f cam_topview_pose;
gmtl::Matrix44f cam_topview_pose_n;



// Transformation matrices applied to plane and camera poses
gmtl::Matrix44f xtransp_mat;
gmtl::Matrix44f xtransn_mat;
gmtl::Matrix44f xrotp_mat;
gmtl::Matrix44f xrotn_mat;

gmtl::Matrix44f ytransp_mat;
gmtl::Matrix44f ytransn_mat;
gmtl::Matrix44f yrotp_mat;
gmtl::Matrix44f yrotn_mat;

gmtl::Matrix44f ztransp_mat;
gmtl::Matrix44f ztransn_mat;
gmtl::Matrix44f zrotp_mat;
gmtl::Matrix44f zrotn_mat;

gmtl::Matrix44f camrotp_mat;
gmtl::Matrix44f camrotn_mat;

gmtl::Matrix44f xrotfix; 
gmtl::Matrix44f ytransfix; 
gmtl::Matrix44f fixed_view_mat;

//|___________________
//|
//| Function Prototypes
//|___________________

void InitMatrices();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawPlane(const float width, const float length, const float height);

//|____________________________________________________________________
//|
//| Function: InitMatrices
//|
//! \param None.
//! \return None.
//!
//! Initializes all the matrices
//|____________________________________________________________________

void InitMatrices()
{
  const float TRANS_AMOUNT = 1.0f;
  const float ROT_AMOUNT   = gmtl::Math::deg2Rad(5.0f); // specified in degs, but get converted to radians

  const float ROT_CAM = gmtl::Math::deg2Rad(90.0f);

  const float COSTHETA = cos(ROT_AMOUNT);
  const float SINTHETA = sin(ROT_AMOUNT);

  const float COSTHETA_CAM = cos(ROT_CAM);
  const float SINTHETA_CAM = sin(ROT_CAM);

  // Move left on X-axis
  xtransp_mat.set(1, 0, 0, TRANS_AMOUNT,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1);
  xtransp_mat.setState(gmtl::Matrix44f::TRANS);

  gmtl::invert(xtransn_mat, xtransp_mat);

  ytransp_mat.set(1, 0, 0, 0,
                  0, 1, 0, TRANS_AMOUNT,
                  0, 0, 1, 0,
                  0, 0, 0, 1);
  ytransp_mat.setState(gmtl::Matrix44f::TRANS);

  gmtl::invert(ytransn_mat, ytransp_mat);

  // Positive Z-Translation
  ztransp_mat.set(1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, TRANS_AMOUNT,
                  0, 0, 0, 1);
  ztransp_mat.setState(gmtl::Matrix44f::TRANS);  

  gmtl::invert(ztransn_mat, ztransp_mat);

  xrotp_mat.set(1,        0,         0, 0,
                0, COSTHETA, -SINTHETA, 0,
                0, SINTHETA,  COSTHETA, 0,
                0,        0,         0, 1);
  xrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);
  gmtl::invert(xrotn_mat, xrotp_mat);

  yrotp_mat.set(COSTHETA, 0, SINTHETA, 0,
                       0, 1,        0, 0,
               -SINTHETA, 0, COSTHETA, 0,
                       0, 0,        0, 1);
  yrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);
  gmtl::invert(yrotn_mat, yrotp_mat);

  // Positive Z-rotation (roll)
  zrotp_mat.set(COSTHETA, -SINTHETA, 0, 0,
                SINTHETA,  COSTHETA, 0, 0,
                       0,         0, 1, 0,
                       0,         0, 0, 1);
  zrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);
  // Negative Z-rotation (roll)
  gmtl::invert(zrotn_mat, zrotp_mat);

  camrotp_mat.set(1,           0,             0, 0,
                 0, COSTHETA_CAM, -SINTHETA_CAM, 0,
                 0, SINTHETA_CAM,  COSTHETA_CAM, 0,
                 0,            0,             0, 1);
  camrotp_mat.setState(gmtl::Matrix44f::ORTHOGONAL);
  gmtl::invert(camrotn_mat, camrotp_mat);
  

  // Inits plane pose
  plane_pose.set(1, 0, 0,  1.0f,
                 0, 1, 0,  0.0f,
                 0, 0, 1,  4.0f,
                 0, 0, 0,  1.0f);
  plane_pose.setState(gmtl::Matrix44f::AFFINE);     // AFFINE because the plane pose can contain both translation and rotation         

  // Inits camera pose and view transform
  cam_pose.set(1, 0, 0,  2.0f,
               0, 1, 0,  1.0f,
               0, 0, 1, 15.0f,
               0, 0, 0,  1.0f);
  cam_pose.setState(gmtl::Matrix44f::AFFINE);            
  gmtl::invert(view_mat, cam_pose);                 // View transform is the inverse of the camera pose

  cam_topview_pose.set(1, 0, 0, 0.0f,
                       0, 1, 0, 0.0f,
                       0, 0, 1, 15.0f,
                       0, 0, 0, 1.0f);
  cam_topview_pose.setState(gmtl::Matrix44f::AFFINE);
  gmtl::invert(cam_topview_pose_n, cam_topview_pose);

  xrotfix.set(1,            0,             0, 0.0f,
              0, COSTHETA_CAM, -SINTHETA_CAM, 0.0f,
              0, SINTHETA_CAM,  COSTHETA_CAM, 0.0f,
              0,            0,             0, 1.0f);
  xrotfix.setState(gmtl::Matrix44f::ORTHOGONAL);

  ytransfix.set(1, 0, 0,   0.0f,
                0, 1, 0, -20.0f,
                0, 0, 1,   0.0f,
                0, 0, 0,   1.0f);
  ytransfix.setState(gmtl::Matrix44f::TRANS);

  fixed_view_mat = xrotfix * ytransfix;

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
  // Modelview matrix
  gmtl::Matrix44f modelview_mat;        // M, as defined in the handout

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//|____________________________________________________________________
//|
//| Viewport 1 rendering: shows the moving camera's view
//|____________________________________________________________________

  glViewport(0, 0, (GLsizei)w_width / 2, (GLsizei)w_height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(CAM_FOV, (float)w_width/(2*w_height), 0.1f, 100.0f);     // Check MSDN: google "gluPerspective msdn"

  // Approach1
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();                          // A good practice for beginner

  // Draws world coordinate frame
  modelview_mat = view_mat;                  // M = C^-1
  glLoadMatrixf(modelview_mat.mData);
  DrawCoordinateFrame(100);

  // Draws plane and its local frame
  modelview_mat *= plane_pose;               // M = C^-1 * T
  glLoadMatrixf(modelview_mat.mData);
  DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);

/*
  // Approach 2 (gives the same results as the approach 1)
  glMatrixMode(GL_MODELVIEW);

  // Draws world coordinate frame
  glLoadMatrixf(view_mat.mData);             // M = C^-1
  DrawCoordinateFrame(10);

  // Draws plane and its local frame
  glMultMatrixf(plane_pose.mData);           // M = C^-1 * T (OpenGL calls build transforms in left-to-right order)
  DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);
*/

//|____________________________________________________________________
//|
//| TODO: Viewport 2 rendering: shows the fixed top-down view
//|____________________________________________________________________

  glViewport(w_width / 2, 0, (GLsizei)w_width / 2, (GLsizei)w_height);
  
  modelview_mat = cam_topview_pose_n * camrotp_mat;
  glLoadMatrixf(modelview_mat.mData);
  DrawCoordinateFrame(100);

  modelview_mat *= plane_pose;               // M = C^-1 * T
  glLoadMatrixf(modelview_mat.mData);
  DrawPlane(P_WIDTH, P_LENGTH, P_HEIGHT);
  DrawCoordinateFrame(3);

  modelview_mat = fixed_view_mat * cam_pose;
  glLoadMatrixf(modelview_mat.mData);
  DrawCoordinateFrame(3);


  glFlush(); // Ensure all OpenGL commands are executed
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
  switch (key) {
//|____________________________________________________________________
//|
//| Plane controls
//|____________________________________________________________________
    
    case 'w': // Forward translation of the plane (positive Z-translation)
      plane_pose = plane_pose * ztransp_mat;
      break;
    case 's': // Backward translation of the plane
      plane_pose = plane_pose * ztransn_mat;
      break;

    // Rotation
    case 'g': // Rolls the plane (+ Z-rot)
      plane_pose = plane_pose * zrotp_mat;
      break;
    case 'f': // Rolls the plane (- Z-rot)
      plane_pose = plane_pose * zrotn_mat;
      break;

    // TODO: Add the remaining controls/transforms
    // x axis
    case 'a':
        plane_pose = plane_pose * xtransp_mat;
        break;
    case 'd':
        plane_pose = plane_pose * xtransn_mat;
        break;
    // y axis
    case 't':
        plane_pose = plane_pose * ytransp_mat;
        break;
    case 'r':
        plane_pose = plane_pose * ytransn_mat;
        break;
    // Rotation
    // x axis
    case 'z':
        plane_pose = plane_pose * xrotp_mat;
        break;
    case 'x':
        plane_pose = plane_pose * xrotn_mat;
        break;
    // y axis
    case 'c':
        plane_pose = plane_pose * yrotp_mat;
        break;
    case 'v':
        plane_pose = plane_pose * yrotn_mat;
        break;

//|____________________________________________________________________
//|
//| Camera controls
//|____________________________________________________________________

    case 'u': // Forward translation of the camera (negative Z-translation - cameras looks in its (local) -Z direction)
      cam_pose = cam_pose * ztransn_mat;
      break;
    case 'j': // Backward translation of the camera
      cam_pose = cam_pose * ztransp_mat;
      break;


    // TODO: Add the remaining controls
    // Translation
    // x axis
    case 'h': 
        cam_pose = cam_pose * xtransn_mat;
        break;
    case 'k': 
        cam_pose = cam_pose * xtransp_mat;
        break;
    // y axis
    case 'l':
        cam_pose = cam_pose * ytransn_mat;
        break;
    case 'o':
        cam_pose = cam_pose * ytransp_mat;
        break;

    // Rotation
    case 'n': 
        cam_pose = cam_pose * zrotn_mat;
        break;
    case 'm': 
        cam_pose = cam_pose * zrotp_mat;
        break;
    case ',': 
        cam_pose = cam_pose * xrotn_mat;
        break;
    case '.': 
        cam_pose = cam_pose * xrotp_mat;
        break;
    case '[':
        cam_pose = cam_pose * yrotn_mat;
        break;
    case ']' : 
        cam_pose = cam_pose * yrotp_mat;
        break;
  }
  gmtl::invert(view_mat, cam_pose);       // Updates view transform to reflect the change in camera transform
  glutPostRedisplay();                    // Asks GLUT to redraw the screen
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
//| Function: DrawPlane
//|
//! \param width       [in] Width  of the plane.
//! \param length      [in] Length of the plane.
//! \param height      [in] Height of the plane.
//! \return None.
//!
//! Draws the plane.
//|____________________________________________________________________

void DrawPlane(const float width, const float length, const float height)
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
  float jet = lima * 1.25;

  // Body dy
  glBegin(GL_QUADS);
  glColor3f(1.0f, 0.0f, 0.0f);

  glVertex3f(-w,  -h, lima);
  glVertex3f( w,  -h, lima);
  glVertex3f( w,   h, lima);
  glVertex3f(-w,   h, lima);

  glVertex3f(-w, -h, -lima);
  glVertex3f(-w,  h, -lima);
  glVertex3f( w, -h, -lima);
  glVertex3f( w,  h, -lima);

  //glVertex3f(w, -h, -lima);
  //glVertex3f(w,  h, -lima);
  //glVertex3f(w,  h,  lima);
  //glVertex3f(w, -h,  lima);

  //glVertex3f(-w, -h, -lima);
  //glVertex3f(-w, -h, lima);
  //glVertex3f(-w,  h, lima);
  //glVertex3f(-w,  h, -lima);

  //glVertex3f(-w, h, -lima);
  //glVertex3f(-w, h, lima);
  //glVertex3f(w, h, lima);
  //glVertex3f(w, h, -lima);

  //glVertex3f(-w, -h, -lima);
  //glVertex3f(w, -h, -lima);
  //glVertex3f(w, -h, lima);
  //glVertex3f(-w, -h, lima);
  //glEnd();

  //// head 
  //glBegin(GL_TRIANGLES);
  //glColor3f(0.0f, 0.0f, 1.0f);

  //glVertex3f(w, h, lima);
  //glVertex3f(w, -h, lima);
  //glVertex3f(w, -h, lima + l);

  //glVertex3f(-w, h, lima);
  //glVertex3f(-w, -h, lima);
  //glVertex3f(-w, -h, lima + l);
  //glEnd();

  //glBegin(GL_QUADS);
  //glColor3f(0.0f, 0.0f, 1.0f);

  //glVertex3f(-w, -h, lima + l);
  //glVertex3f(-w, h, lima);
  //glVertex3f(w, h, lima);
  //glVertex3f(w, -h, lima + l);

  //glVertex3f(-w, -h, lima + l);
  //glVertex3f(-w, -h, lima);
  //glVertex3f(w, -h, lima);
  //glVertex3f(w, -h, lima + l);
  //glEnd();

  //// wing
  //glBegin(GL_TRIANGLES);
  //glColor3f(0.0f, 1.0f, 0.0f);

  //glVertex3f(w, -h, -lima);
  //glVertex3f(whiskey, -h, -lima);
  //glVertex3f(w, -h, lima + l);

  glVertex3f(-w, -h, -lima);
  glVertex3f(-whiskey, -h, -lima);
  glVertex3f(-w, -h, lima + l);
  glEnd();

  // tail l  
  glBegin(GL_TRIANGLES);
  glColor3f(0.0f, 1.0f, 1.0f);

  glVertex3f(tango, h, -lima);
  glVertex3f(tango, hotel, -lima);
  glVertex3f(tango, h, tail_lima);

  glVertex3f(-tango, h, -lima);
  glVertex3f(-tango, hotel, -lima);
  glVertex3f(-tango, h, tail_lima);
  glEnd();

  glBegin(GL_QUADS);
  glColor3f(0.0f, 1.0f, 1.0f);

  glVertex3f(-tango, h, tail_lima);
  glVertex3f(-tango, hotel, -lima);
  glVertex3f(tango, hotel, -lima);
  glVertex3f(tango, h, tail_lima);

  glVertex3f(-tango, h, -lima);
  glVertex3f(-tango, hotel, -lima);
  glVertex3f(tango, hotel, -lima);
  glVertex3f(tango, h, -lima);
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
  InitMatrices();

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(w_width, w_height);
  
  glutCreateWindow("Plane Episode 1");

  glutDisplayFunc(DisplayFunc);
  glutReshapeFunc(ReshapeFunc);
  glutKeyboardFunc(KeyboardFunc);
  
  InitGL();

  glutMainLoop();

  return 0;
}