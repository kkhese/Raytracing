// Ass2.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Header Files
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "mymodel.h"

using namespace std;

// Construct ray for each pixel
void RayConstruction(float i, float j, float P0[3], float V[3])
{
    // Xc, Yc Mapping Screen on film 
    float Xc, Yc;
    Xc = ((xrmost - xlmost) * (j / (COLS - 1))) + xlmost;
    Yc = ((ymax - ymin) * (i / (ROWS - 1))) + ymin;    
    // Origin of World Coordinate
    float Origin[4] = { 0.0, 0.0, 0.0, 1.0 };
    // rawP is world coordinate value for camera origin
    float rawP[4];
    matrixcal2(Mcw, Origin, rawP);
    // Homogenius to 3D vector, P0 is the camera location in world coordinates
    P0[0] = rawP[0]; P0[1] = rawP[1]; P0[2] = rawP[2];
    // one selected point of image plane(film) by i and j
    float rawP1[4] = { Xc, Yc, focal, 1.0 };
    float P1[4] = { 0.0, };
	// transform the selected poit of film to world coordinates
    matrixcal2(Mcw, rawP1, P1);
    // Ray vector for selected point by i and j
    float rawV[3] = { 0.0, };
    for (int i = 0; i < 3; i++)
    {
        rawV[i] = P1[i] - P0[i];
    }
	// V is normalized ray vector
    V[0] = rawV[0]/Length(rawV);
    V[1] = rawV[1]/Length(rawV);
    V[2] = rawV[2]/Length(rawV);
}

// Decide shading value C, from N, P, kd
unsigned char Shading(float P[3], float N[3], float kd)
{
	// C should be unsigned char for RAW image
    unsigned char C;
	// Light Vector from P to LRP, P is input
    float L[3];
    float rawL[3] = { LRP[0] - P[0], LRP[1] - P[1], LRP[2] - P[2] };
	// Normalizing light vector
    Length(rawL);
    L[0] = rawL[0] / Length(rawL);
    L[1] = rawL[1] / Length(rawL);
    L[2] = rawL[2] / Length(rawL);
	// Equation for obtaining shading
    float Ctemp = (Ip * kd * (N[0] * L[0] + N[1] * L[1] + N[2] * L[2]));
    // if angle bet. light and normal is smaller than 90, use calculated shading value
    if (Ctemp > 1.0) {
        C = (unsigned char)(unsigned int)(Ip * kd * (N[0] * L[0] + N[1] * L[1] + N[2] * L[2]));
    }
	// if angle bet. light and normal is greater than or equal with 90.
	// force total black.
    else {
        C = (unsigned char) 1;
    }
    return (C);
}

// Ray and Sphere intersection
float RSintersection(float P0[3], float V[3], float N[3]) {
	// Camera to sphear center vector
    float L[3] = { obj1.x - P0[0], obj1.y - P0[1], obj1.z - P0[2] };
    // tca and d was copied from lecture note tca = (L dot V), d^2 = L^2 - tca^2
	// V is normalized so the same with D of lecture note
	float tca = L[0] * V[0] + L[1] * V[1] + L[2] * V[2];
	// dd is d^2
    float dd = (Length(L) * Length(L)) - (tca * tca);
	// rr is r^2: r is radius
	float rr = obj1.radius * obj1.radius;
	// tch definition by the lecture note
	float thc = sqrt(rr - dd);
	// Initialize t value as NULL
	float t = tca;
	// If r is lager than or equal with d, it meet
	if ( dd <= rr) {
		// we only need near one between t0 and t1
        t = tca - thc;
		// Normal Vector is from origin of sphere to intersection point
        N[0] = (P0[0] + t * V[0]) - obj1.x;
        N[1] = (P0[1] + t * V[1]) - obj1.y;
        N[2] = (P0[2] + t * V[2]) - obj1.z;
		// Normalizing N vector
        float Nlength = Length(N);
        N[0] = N[0] / Nlength;
        N[1] = N[1] / Nlength;
        N[2] = N[2] / Nlength;
    }
	// If there is no intersection, set t as negative so we can neglect later.
	// and no normalized vector
    else {
        t = NULL;
        N[0] = NULL;
        N[1] = NULL;
        N[2] = NULL;
    } 
    return (t);
}

// Ray and polygon intersection
float RPintersection(float P0[3], float V[3], float N[3]) 
{
    // # of vertices of plygon = # of row of obj2(POLY4).v
    const int NOV = sizeof(obj2.v) / sizeof(obj2.v[0][0]) / 3;
	// Calculate 2 vecters V0V1 and V0V2
    float V1[3] = { obj2.v[1][0] - obj2.v[0][0], obj2.v[1][1] - obj2.v[0][1], obj2.v[1][2] - obj2.v[0][2] };
    float V2[3] = { obj2.v[2][0] - obj2.v[0][0], obj2.v[2][1] - obj2.v[0][1], obj2.v[2][2] - obj2.v[0][2] };
    // From two vectors, calculate N vector of plane
	crossX(V2, V1, N);
	// Obtain D value of plane equation
    float D = -(obj2.v[0][0] * N[0] + obj2.v[0][1] * N[1] + obj2.v[0][2] * N[2]);
    // Initialize t as NULL
	float t = NULL;
	// If plane normal vector and ray vector are perpendicular, ignore that case.
    if ((N[0] * V[0] + N[1] * V[1] + N[2] * V[2]) == 0.0)
    {
        t = NULL;
        N[0] = NULL;
        N[1] = NULL;
        N[2] = NULL;
    }
    else {
	// If N and ray are not perpendicular, calcuate t
        t = -( (N[0] * P0[0] + N[1] * P0[1] + N[2] * P0[2]) + D) / (N[0] * V[0] + N[1] * V[1] + N[2] * V[2]);
    }
	// Intersection point
    float rayPoint[3] = { P0[0] + t * V[0], P0[1] + t * V[1], P0[2] + t * V[2] };
	// projecting object to 2D plane
    float obj2D[NOV][2];
	// Project intersection point to 2D plane also
    float ray2D[2];
	// If we have intersection bet. ray and plane
    if (t != NULL)
    {
        // Converting plane to 2D image by Normal vector
        // create projected image by dominent direction of normal vector 
		// When x compenent of Normal vector is dominent, kill x value
		if ((N[0] >= N[1]) && (N[0] >= N[2])) {
            for (int i = 0; i < NOV; i++)
            {
                obj2D[i][0] = obj2.v[i][1];
                obj2D[i][1] = obj2.v[i][2];
                ray2D[0] = rayPoint[1];
                ray2D[1] = rayPoint[2];
                // bring ray-plane intersection point to origin
                obj2D[i][0] = obj2D[i][0] - ray2D[0];
                obj2D[i][1] = obj2D[i][1] - ray2D[1];
            }
        }
		// When y compenent of Normal vector is dominent, kill y value
        else if ((N[1] >= N[0]) && (N[1] >= N[2])) {
            for (int i = 0; i < NOV; i++)
            {
                obj2D[i][0] = obj2.v[i][2];
                obj2D[i][1] = obj2.v[i][0];
                ray2D[0] = rayPoint[2];
                ray2D[1] = rayPoint[0];
                // bring ray-plane intersection point to origin
                obj2D[i][0] = obj2D[i][0] - ray2D[0];
                obj2D[i][1] = obj2D[i][1] - ray2D[1];
            }
        }
		// When z compenent of Normal vector is dominent, kill z value
		else {
            for (int i = 0; i < NOV; i++)
            {
                obj2D[i][0] = obj2.v[i][0];
                obj2D[i][1] = obj2.v[i][1];
                ray2D[0] = rayPoint[0];
                ray2D[1] = rayPoint[1];
                // bring ray-plane intersection point to origin
                obj2D[i][0] = obj2D[i][0] - ray2D[0];
                obj2D[i][1] = obj2D[i][1] - ray2D[1];
            }
        }
		// vertex i to vertex k represent each line of polygon
		// i is indicator for 1st vertex and k is indicater for 2nd vertex  
        int k = 0;
        // counting number of intersection with each line and y=0
        int countInter = 0;
		// we check for every line of polygon
        for (int i = 0; i < NOV; i++)
        {
			// calculate k for given i, it is i+1 except last vertex.
			// If i indicate last vertex of polygon, k should be 0
            k = (i + 1) % (NOV);
            // y = ax + c : Line equation
            float a = 0.0;          // slope of line from neighboring 2 vertices
            float c = 0.0;          // meeting point onto 2nd axis of projected 2D dimension.
            float MeetPointx = 0.0; // edge line and x-axis meeting position
            
            // y=ax+c => y is 2nd axis, x is 1st axis. Not always the same with x,y, and z of xyz coordinates 
            // slope a is (obj2D[i][1] - obj2D[k][1]) / (obj2D[i][0] - obj2D[k][0])
            // if slope is inf, meeting point always exists at x position of the edge
            // if abs(denominator) is small enough(<0.0002) to consider slope as infinite 
            if (obj2D[i][0] - obj2D[k][0] < 0.0002 || obj2D[i][0] - obj2D[k][0] > -0.0002) {
				// it means x value of 2 vertices are the same
				// if x values are saying on negative side of x axis
				// set meeting position to negative value so it can be neglected when couting # of intersection
                if ((0.0 <= obj2D[i][1] && 0.0 < obj2D[k][1]) ||
                    (0.0 >= obj2D[i][1] && 0.0 > obj2D[k][1]))
                {
                    MeetPointx = -100.0;
                }
				// Otherwise, if line is located x positive side with inf. slope,
				// Meetpointx is either obj2D[i][0] or obj2D[k][0]. obj2D[k][0] was used 
                else
                {
                    MeetPointx = obj2D[k][0];
                }
            }
            // if slope is 0, meet only when y position is 0, otherwise it parallel with x axis(never meet) 
            // abs(numerator) is small enough(<0.0002) to consider as 0
			// For simplisity, even though y position is 0, we consider it as never meet condition
            else if ((obj2D[i][1] - obj2D[k][1]) < 0.0002 && (obj2D[i][1] - obj2D[k][1])  > -0.0002 ) {
				MeetPointx = -100.0;
            }
			// Otherwise it is line having non-zero finite slope! 
            else {
				// calculating slope first
                a = (obj2D[i][1] - obj2D[k][1]) / (obj2D[i][0] - obj2D[k][0]);
                // y = ax + c   <===  using x = obj2D[i][0], y = obj2D[i][1] for getting c 
                c = obj2D[i][1] - a * obj2D[i][0];
                // check if there is an intersection with y=ax+c and y = 0 
                // 0 = ax + c, so intersection is point is ( (-c/a), 0.0 )
                MeetPointx = -(c / a);
            }
			// Even though y=ax+c meets with y=0, we have to consider +x axis only
            // If extended polygon edge are meeting at the negative side of x axis
            // There is no intersection. They are actually not meeting
            if (MeetPointx <= 0)
            {
				// Don't count
                countInter = countInter;
            }
            // Only extended polygon edge meet at the positive x axis, 
            // and intersection point is out of boundary of edge line
			// They are actually not meeting
            else if (   (MeetPointx <= obj2D[i][0] && MeetPointx < obj2D[k][0]) ||
                        (MeetPointx >= obj2D[i][0] && MeetPointx > obj2D[k][0])     )
            {
				// don't count
                countInter = countInter;
            }
            // Otherwise, it is ray and polygon line is meeting, increase counter
            else {
                countInter++;
            }
        }
        // if # of intersection points are odd, intersection point is inside polygon 
        if (countInter % 2 == 1)
        {
            return(t);
        }
        // if # of intersection points are even, intersection point is outside polygon
        else
        {
            N[0] = NULL;
            N[1] = NULL;
            N[2] = NULL;
            return(NULL);
        }
	
    }
	else return(NULL);
}

// Ray - object intersection
int ROintersection(float P0[3], float V[3], float P[3], float N[3]) {
    float N1[3], N2[3];		// Normal for Sphere and Polygon
    float kd1 = obj1.kd;	// kd of sphere
    float kd2 = obj2.kd;	// kd of polygon
    // find t for sphere, and get normal vector from the intersetion
	float t1 = RSintersection(P0, V, N1);
	// find t for polyong, and get normal vector of polygon
    float t2 = RPintersection(P0, V, N2);
	// If no intersection bet. ray and obj.s at all
    if (t1 == NULL && t2 == NULL)
    {
        return(NULL);
    }
	// If ray intersect only with sphere
    else if (t2 == NULL) {
		// Consider intersection with sphere only
        for (int i = 0; i < 3; i++)
        {
            P[i] = P0[i] + t1 * V[i];
            N[i] = N1[i];
        }
        // to select kd1 for kd
        return(1);
    }
	// If ray intersect only with polygon
    else if (t1 == NULL) {
		// Consider intersection with polygon only
        for (int i = 0; i < 3; i++)
        {
            P[i] = P0[i] + t2 * V[i];
            N[i] = N2[i];
        }
        // to select kd2 for kd
        return(2);
    }
	// If ray intersect with both, but sphere is closer
    else if (t1 < t2) {
		// Consider intersection with sphere only
        for (int i = 0; i < 3; i++)
        {
            P[i] = P0[i] + t1 * V[i];
            N[i] = N1[i];
        }
        // to select kd1 for kd
        return(1);
    }
	// If ray intersect with both, but polygon is closer
    else {
		// Consider intersection with polygon only
        for (int i = 0; i < 3; i++)
        {
            P[i] = P0[i] + t2 * V[i];
            N[i] = N2[i];
        }
        // to select kd2 for kd
        return(2);
    }
}

// Raytracing function for obtaining correct shading calue
unsigned char RayTracing(float P0[3], float V[3]) {
    float P[3], N[3];
    float kd=0.0;
	// P0 camera location, V is ray, P is intersection point, N is normal vector
	// found is 1 if intersection is with sphere, and found is 2 if intersection is with polygon
    int found = ROintersection(P0, V, P, N);
	// Center of Sphere
    float center[3] = { obj1.x, obj1.y, obj1.z };
    unsigned char c;
	// In case of intersected by sphere
    if (found == 1) {
        c = Shading(P, N, obj1.kd);				// light vectors are intersection to LRP
//		c = Shading(center, N, obj1.kd);		// light vectors are all parallel with origin of sphere to LRP
		return (c);
    }
	// In case of intersected by polygon
    else if (found == 2) {
        c = Shading(P, N, obj2.kd);
        return (c);
    }
	// In case of no intersection
    else
    {
        return (NULL);
    }
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////     Main function starts here   /////////////////////
////////////////////////////////////////////////////////////////////////////////

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Important!!!!!!!!!!!!!!!!!!!!!!!!!!!///////////
// To proper run, you need to add command argument "1 [file name].raw"
int main(int argc, char **argv)
{
	// Ready to write RAW image file
    FILE * fout;
    int n = 0;
    if (fopen_s(&fout, argv[2], "wb") != 0) {
        fprintf(stderr, "Error: Can't open output image file %s\n", argv[2]);
    }
	// Get T and invT matrix @ T[3], invT[3]
    float T[4][4] = { {1.0, 0.0, 0.0, -VRP[0]}, {0.0, 1.0, 0.0, -VRP[1]}, {0.0, 0.0, 1.0, -VRP[2]}, {0.0, 0.0, 0.0, 1.0} };
    float invT[4][4] = { {1.0, 0.0, 0.0, VRP[0]}, {0.0, 1.0, 0.0, VRP[1]}, {0.0, 0.0, 1.0, VRP[2]}, {0.0, 0.0, 0.0, 1.0} };
    // Get R and inverted R matrices from VUP and VPN
    float R[4][4];
    float invR[4][4];
    uvn(VUP, VPN, R, invR);
    // Modify Mwc and Mcw from R and T
    matrixcal(R, T, Mwc);
    matrixcal(invT, invR, Mcw);
    // Origin in Cameraview
    float P0[3] = { 0.0, };
	// Initialize ray vector
    float V[3] = { 0.0, };
	// variables for RAW image pixel value
    unsigned char C;
 
    // Initialize global data structures
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            //Get ray vectors for each pixel
            RayConstruction((float)i, (float)j, P0, V);
			// If intersection exists, obtain shading value
			if ((C = RayTracing(P0, V) != NULL)) {
                img[i][j] = (unsigned char) RayTracing(P0, V);
				// need for filter
//                img2[i][j] = (unsigned char)RayTracing(P0, V);
            }
			// If no intersection, set that pixel to default (AUTO) value 
            else {
                img[i][j] = (unsigned char) AUTO;
				// need for filter
//                img2[i][j] = (unsigned char)AUTO;
            }
        }
    }
	// Image correction / Not needed
//    for (int i = 1; i < ROWS-1; i++) {
//        for (int j = 1; j < COLS-1; j++) {
//            if ( img[i][j] == AUTO && ( img2[i-1][j] == 1 || img2[i + 1][j] == 1))  {
//                img[i][j] = (unsigned char) 1;
//            }
//            if (img[i][j] == AUTO && (img2[i][j - 1] == 1 || img2[i][j + 1] == 1)) {
//                img[i][j] = (unsigned char)1;
//            }
//        }
//    }

	// Print out writing status 
    printf(" ... Save the output image\n");
    n = fwrite(img, sizeof(char), ROWS * COLS, fout);
    fclose(fout);
    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
