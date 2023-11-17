/* IMPORTANT: After you download this file, you should rename it
	to "mymodel.h"
*/

/* Definition of the structure for Sphere */
typedef struct {
	float x, y, z;	/* center of the circle */
	float radius;	/* radius of the circle */
	float kd;	/* diffuse reflection coefficient */
} SPHERE;

/* Definition of Polygon with 4 edges */
typedef struct {
	float v[4][3];	/* list of vertices */
	float N[3];	/* normal of the polygon */
	float kd;	/* diffuse reflection coefficient */
} POLY4;

/* create a spherical object */
SPHERE obj1 = {	1.0, 1.0, 1.0,	/* center of the circle */
				1.0,			/* radius of the circle */
				0.75 };			/* diffuse reflection coefficient */

/* create a polygon object */
POLY4 obj2 = {  0.0, 1.2, 0.0,	/* v0 */
				0.0, 1.2, 2.0,	/* v1 */
				2.0, 0.8, 2.0,	/* v2 */
				2.0, 0.8, 0.0,	/* v3 */
				0.0, 1.0, 0.0,	/* normal of the polygon */
				0.8f };		/* diffuse reflection coefficient */

/* definition of the image buffer */
#define ROWS 512
#define COLS 512
// Define background color
#define AUTO 180
// 2 image files, img2 used for img filtering 
unsigned char img[ROWS][COLS] = { AUTO, };
// unsigned char img2[ROWS][COLS] = { 1, };

/* definition of window on the image plane in the camera coordinates */
/* They are used in mapping (j, i) in the screen coordinates into */
/* (x, y) on the image plane in the camera coordinates */
/* The window size used here simulates the 35 mm film. */
float xlmost = 0.0175f;		// x left most is Positive side in Camera view
float xrmost = -0.0175f;	// x right most is Negative side in Camera view

float ymin = 0.0175f;		// y is fliped since RAW file writing from top to bottom
float ymax = -0.0175f;		// which is opposite with y positive direction

/* definition of the camera parameters */
//float VRP[3] = { 1.0, 2.0, 3.5 };			// Original camera location: Too close
float VRP[3] = { 1.0, 3.0, 6.0 };			// Camera was pulled back, direction preserved
float VPN[3] = { 0.0, -1.0, -2.5 };
//float VPN[3] = { 1.0, -0.5, 1.5 };
float VUP[3] = { 0.0, 1.0, 0.0 };

float focal = 0.05f;							/* focal length simulating 50 mm lens */

/* definition of light source */
float LRP[3] = { -10.0, 10.0, 2.0 };		/* light position */
float Ip = 200.0;							/* intensity of the point light source */

/* === transformation matrices (to be constructed) === */
/* Initialize Transformation from the world to the camera coordinates */
float Mwc[4][4] =
{ 1.0, 0.0, 0.0, 0.0,
 0.0, 1.0, 0.0, 0.0,
 0.0, 0.0, 1.0, 0.0,
 0.0, 0.0, 0.0, 1.0 };

/* Initialize Transformation from the camera to the world coordinates */
float Mcw[4][4] =
{ 1.0, 0.0, 0.0, 0.0,
 0.0, 1.0, 0.0, 0.0,
 0.0, 0.0, 1.0, 0.0,
 0.0, 0.0, 0.0, 1.0 };

////////////////////// Fundamental functions were added to header file ////////////////////////
// Obtain magnitude of vecter => returning to nor
float Length(float vector[3])
{
	float abs = vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2];
	float mag = sqrt(abs);
	return mag;
}

// |Vec2 X Vec1| = cross ( cross is normalized vector ) : Order is important!!!!!!!!
// Not |Vec1 X Vec2|, but |Vec2 X Vec1|
void crossX(float vec1[3], float vec2[3], float cross[3]) {
	// cross = Vec2 X Vec 1
	cross[0] = vec2[1] * vec1[2] - vec2[2] * vec1[1];
	cross[1] = (vec2[2] * vec1[0] - vec2[0] * vec1[2]);
	cross[2] = vec2[0] * vec1[1] - vec2[1] * vec1[0];
	// normalized cross, |cross|
	float mag = Length(cross);
	cross[0] = cross[0] / mag;
	cross[1] = cross[1] / mag;
	cross[2] = cross[2] / mag;
}

// out = A X B ( 4by4 X 4by4 matrix calculation )
void matrixcal(float A[4][4], float B[4][4], float out[4][4]) {
	float component = 0.0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			component = 0.0;
			for (int k = 0; k < 4; k++) {
				component += A[i][k] * B[k][j];
			}
			out[i][j] = component;
		}
	}
}

// out = A X B ( 4by4 X 1by4 matrix calculation )
void matrixcal2(float A[4][4], float B[4], float out[4]) {
	float component = 0.0;
	for (int i = 0; i < 4; i++) {
		component = 0.0;
		for (int j = 0; j < 4; j++) {
			component += A[i][j] * B[j];
		}
		out[i] = component;
	}
}

// out = transpose matrix of A, used for obtaining inverted matrix
void Transpose(float A[4][4], float out[4][4]) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			out[j][i] = A[i][j];
		}
	}
}

// Obtaining u,v,n unit vector & rotation, inverted rotation matrix from 2 vectors (VPN, VUP) 
void uvn(float VUP[3], float VPN[3], float R[4][4], float invR[4][4])
{
	float length = Length(VPN);
	float u[3], v[3], n[3];
	// n vector
	n[0] = VPN[0] / length;
	n[1] = VPN[1] / length;
	n[2] = VPN[2] / length;
	// u and v vectors
	crossX(VPN, VUP, u);
	crossX(u, n, v);
	// Rotation vetor
	for (int i = 0; i < 3; i++)
	{
		R[0][i] = u[i];
		R[1][i] = v[i];
		R[2][i] = n[i];
		R[3][i] = 0.0;
	}
	for (int j = 0; j < 3; j++)
	{
		R[j][3] = 0.0;
	}
	R[3][3] = 1.0;
	// Inverted rotation vector
	Transpose(R, invR);
}
////////////////////////////////// End of Header file //////////////////////////////////////