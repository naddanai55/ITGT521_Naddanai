// CMPS 415/515, University of Louisiana at Lafayette
//
// Example 1.a : draw a horizontal line in a simulated frame buffer
//               and display it using OpenGL
//
// NOTE: No permission is given for distribution beyond
//       the 415/515 class, of this file or any derivative works.
//

#include <GL/glut.h>
#define WIDTH 400		// width of window (also frame buffer's width)
#define HEIGHT 300		// height of window (also frame buffer's height)
static GLubyte frame_buffer[HEIGHT][WIDTH][3];

/*
	frame_buffer simulates a frame buffer with 3 bytes per pixel (24 bit RGB).

	The first index of "frame_buffer" refers to the y-coordinate (0 to HEIGHT-1).
	The second index of "frame_buffer" refers to the x-coordinate (0 to WIDTH-1).
	The third index of "frame_buffer" selects the R, G, or B byte.
	Each element has value 0 to 255, with 0 being minimum intensity and 255 being max.

	Example [setting the pixel at (100,10) to red]:
	frame_buffer[10][100][0] = 255;
	frame_buffer[10][100][1] = 0;
	frame_buffer[10][100][2] = 0;
*/

/* Called when a display event occurs in GLUT: */
void display(void) {
	/*
		Set the raster position to the lower-left corner to avoid a problem 
		(with glDrawPixels) when the window is resized to smaller dimensions.
	*/
	glRasterPos2i(-1,-1);

	// Write the information stored in "frame_buffer" to the color buffer
	glDrawPixels(WIDTH, HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, frame_buffer);
	glFlush();
}


int main(int argc, char **argv) {
	int i;

	// Draw a blue horizontal line from (50,25) to (249,25) into the simulated frame buffer
	for (i = 0; i < 200; i++) {
		frame_buffer[25][50+i][0] = 0;
		frame_buffer[25][50+i][1] = 0;
		frame_buffer[25][50+i][2] = 255;
	}
	
	// GLUT initialization:
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow("Frame Buffer Example");

	glutDisplayFunc(display);

	// Start GLUT event processing loop:
	glutMainLoop();

	return 1;
}
