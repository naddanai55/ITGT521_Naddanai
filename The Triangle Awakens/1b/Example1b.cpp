// CMPS 415/515, University of Louisiana at Lafayette
//
// Example 1.b : set pixels to red where the mouse clicks
//
// NOTE: No permission is given for distribution beyond
//       the 415/515 class, of this file or any derivative works.
//

#include <stdio.h>
#include <GL/glut.h>
#define WIDTH 400		
#define HEIGHT 300	
static GLubyte frame_buffer[HEIGHT][WIDTH][3];

/*
   see the description in Example 1.a
*/

/* Called when mouse button pressed: */
void mousebuttonhandler(int button, int state, int x, int y)
{
  printf("Mouse button event, button=%d, state=%d, x=%d, y=%d\n", button, state, x, y);

  // set a pixel's red color value when left mouse button is pressed down:
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    frame_buffer[HEIGHT-y-1][x][0] = 255;            

  // cause a display event to occur for GLUT:
  glutPostRedisplay();
}

/* Called by GLUT when a display event occurs: */
void display(void) {

	/*	Set the raster position to the lower-left corner to avoid a problem 
		(with glDrawPixels) when the window is resized to smaller dimensions.*/
	glRasterPos2i(-1,-1);

	// Write the information stored in "frame_buffer" to the color buffer
	glDrawPixels(WIDTH, HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, frame_buffer);
	glFlush();
}

int main(int argc, char **argv) {

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow("Frame Buffer Example");

	// Specify which functions get called for display and mouse events:
	glutDisplayFunc(display);
    glutMouseFunc(mousebuttonhandler);

	glutMainLoop();

	return 1;
}