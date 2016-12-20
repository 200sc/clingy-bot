
#include "stdafx.h"
#include <iostream>
#include <limits>
#include <math.h>
using namespace std;

#include <Windows.h>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <NuiImageCamera.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#define COLOR_WIDTH 640    
#define COLOR_HEIGHT 480    
#define DEPTH_WIDTH 320    
#define DEPTH_HEIGHT 240    
// There are three channels: R,G,B
#define CHANNEL 3
#define aStarModsMax 32

BYTE buf[DEPTH_WIDTH * DEPTH_HEIGHT * CHANNEL];
BYTE buf2[DEPTH_WIDTH * DEPTH_HEIGHT * CHANNEL];
BYTE buf3[COLOR_WIDTH * COLOR_HEIGHT * CHANNEL];
bool grid[DEPTH_WIDTH][DEPTH_HEIGHT];

int aStarMods[aStarModsMax] = {
	// Each set of eight represents a sparse circle
	// which has eight points one can travel to. 
	//0, -2,
	//1, -1,
	//2, 0,
	//1, 1,
	//0, 2,
	//-1, 1,
	//-2, 0,
	//-1, -1,
	//
	1, -3,
	3, -1,
	3, 1,
	1, 3,
	-1, 3,
	-3, 1,
	-3, -1,
	-1, -3,
	//
	//2, -5,
	//5, -2,
	//5, 2,
	//2, 5,
	//-2, 5,
	//-5, 2,
	//-5, -2,
	//-2, -5,
	//
	5, -13,
	13, -5,
	13, 5,
	5, 13,
	-5, 13,
	-13, 5,
	-13, -5,
	-5, -13
};

struct Vector {
	int x1;
	int y1;
	int x2;
	int y2;
	bool update;
};

const NUI_IMAGE_FRAME * getFrame(HANDLE h)
{
	const NUI_IMAGE_FRAME * pImageFrame = NULL;
	HRESULT hr = NuiImageStreamGetNextFrame(h, 0, &pImageFrame);
	if (FAILED(hr))
	{
		std::cout << "Get Image Frame Failed" << std::endl;
		return NULL;
	}
	return pImageFrame;
}

// Credit to these kinect-interfacing code bits to 
// hbezlls via http://stackoverflow.com/questions/13876366/
int drawColor(INuiFrameTexture * pTexture, IplImage* color)
{
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0)
	{
		BYTE * pBuffer = (BYTE*)LockedRect.pBits;
		cvSetData(color, pBuffer, LockedRect.Pitch);
	}
	return 0;
}

int drawDepth(INuiFrameTexture * pTexture, IplImage* depth)
{
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0)
	{
		USHORT * pBuff = (USHORT*)LockedRect.pBits;
		for (int i = 0; i < DEPTH_WIDTH * DEPTH_HEIGHT; i++)
		{
			BYTE index = pBuff[i] & 0x07;
			USHORT realDepth = (pBuff[i] & 0xFFF8) >> 3;
			BYTE scale = 255 - (BYTE)(256 * realDepth / 0x0fff);
			buf[CHANNEL * i] = buf[CHANNEL * i + 1] = buf[CHANNEL * i + 2] = 0;
			switch (index)
			{
			case 0:
				buf[CHANNEL * i] = scale / 2;
				buf[CHANNEL * i + 1] = scale / 2;
				buf[CHANNEL * i + 2] = scale / 2;
				break;
			case 1:
				buf[CHANNEL * i] = scale;
				break;
			case 2:
				buf[CHANNEL * i + 1] = scale;
				break;
			case 3:
				buf[CHANNEL * i + 2] = scale;
				break;
			case 4:
				buf[CHANNEL * i] = scale;
				buf[CHANNEL * i + 1] = scale;
				break;
			case 5:
				buf[CHANNEL * i] = scale;
				buf[CHANNEL * i + 2] = scale;
				break;
			case 6:
				buf[CHANNEL * i + 1] = scale;
				buf[CHANNEL * i + 2] = scale;
				break;
			case 7:
				buf[CHANNEL * i] = 255 - scale / 2;
				buf[CHANNEL * i + 1] = 255 - scale / 2;
				buf[CHANNEL * i + 2] = 255 - scale / 2;
				break;
			}
		}
		cvSetData(depth, buf, DEPTH_WIDTH * CHANNEL);
	}
	
	return 0;
}

bool InColorRange(byte r1, byte g1, byte b1, byte r2, byte g2, byte b2, int range) {
	return (r1 > r2 - range && r1 < r2 + range &&
		b1 > b2 - range && b1 < b2 + range &&
		g1 > g2 - range && g1 < g2 + range);
}

struct Vector drawMoveGrid(INuiFrameTexture * cTexture, INuiFrameTexture * dTexture, IplImage* gridImage, IplImage* colorTestImage)
{
	// Process dTexture into the gridImage

	int targetDepthIndex = 0;
	int targetDepthX = 0;
	struct Vector targetVector;
	targetVector.update = false;

	NUI_LOCKED_RECT LockedRect2;
	cTexture->LockRect(0, &LockedRect2, NULL, 0);
	if (LockedRect2.Pitch != 0)
	{
		BYTE * pBuff = (BYTE*)LockedRect2.pBits;

		int xSum = 0;
		int ySum = 0;
		int pixels = 0;

		//for (int i = 0; i < 10000000; i++)
		//{
		//	BYTE test = pBuff[i];
		//}

		for (int x = 0; x < COLOR_WIDTH; x++)
		{
			for (int y = 0; y < COLOR_HEIGHT; y++)
			{
				int i = (y*COLOR_WIDTH + x) * 4;
				BYTE r = pBuff[i];
				BYTE g = pBuff[i + 1];
				BYTE b = pBuff[i + 2];
				if (InColorRange(r, g, b, 170, 140, 120, 40))
				{
					int j = (y*COLOR_WIDTH + x) * CHANNEL;
					buf3[j] = 255;
					buf3[j + 1] = 255;
					buf3[j + 2] = 255;
					xSum += x;
					ySum += y;
					pixels++;
				}
				else {
					int j = (y*COLOR_WIDTH + x) * CHANNEL;
					buf3[j] = 0;
					buf3[j + 1] = 0;
					buf3[j + 2] = 0;
				}
			}
		}
		if (pixels > 0)
		{
			xSum = xSum / pixels;
			ySum = ySum / pixels;
			int k = (ySum*COLOR_WIDTH + xSum) * CHANNEL;
			buf3[k] = 0;
			buf3[k + 1] = 0;
			buf3[k + 2] = 255;

			targetDepthX = xSum / 2;
			int depthY = ySum / 2;

			targetDepthIndex = (depthY*DEPTH_WIDTH + targetDepthX) * CHANNEL;
		}
		cvSetData(colorTestImage, buf3, COLOR_WIDTH * 3);
	}

	NUI_LOCKED_RECT LockedRect;
	dTexture->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0)
	{
		// Get the minimum depth in each column
		USHORT * pBuff = (USHORT*)LockedRect.pBits;
		for (int x = 0; x < DEPTH_WIDTH; x++)
		{
			USHORT minDepth = 65000;
			// Start y at a higher value to ignore certain rows
			// Just doing one row right now as a test
			int middle = DEPTH_HEIGHT / 2;
			for (int y = middle - 1; y < middle; y++)
			{

				// pBuff is a single array representing a double array.
				// To get to the appropriate row we multiply y by Width.
				int i = y*DEPTH_WIDTH + x;
				USHORT coordDepth = pBuff[i];

				// Too far / unknown
				if (coordDepth == 0x0FFF || coordDepth == 0x1FFF || coordDepth == 0x0000) {
					continue;
				}
				//char str[30];
				//_itoa(int(pBuff[i]), str, 10);
				//cout << str << endl;
				//sprintf(str, "Depth pre-shift: %d", pBuff[i]);
				//sprintf(str, "Depth after shift: %d", coordDepth);
				if (coordDepth < minDepth)
				{
					minDepth = coordDepth;
					//char str[30];
					//_itoa(int(minDepth), str, 10);
					//cout << str << endl;
				}
			}
			//Need to convert minDepth to int between 0 and DEPTH_HEIGHT.
			//Unsigned shorts have a range from 0 to 65535
			//We'll assume that the kinect uses that depth range until 
			//we find out otherwise
			//char buffer[30];
			//_itoa(int(minDepth), buffer, 10);
			//cout << buffer << endl;
			int minDepthIndex = ((int)minDepth) / (65535 / DEPTH_HEIGHT);
			//_itoa(int(minDepthIndex), buffer, 10);
			//cout << buffer << endl;

			// Values higher than the min depth are invalid
			for (int y = minDepthIndex; y < DEPTH_HEIGHT; y++)
			{
				grid[x][y] = false;
				//int y2 = DEPTH_HEIGHT - y;
				int i = CHANNEL * (y*DEPTH_WIDTH + x);

				// Set the RGB of this pixel to black
				buf2[i] = 0;
				buf2[i + 1] = 0;
				buf2[i + 2] = 0;
			}
			// Other values are valid
			for (int y = minDepthIndex - 1; y >= 0; y--)
			{
				//int y2 = DEPTH_HEIGHT - y;
				int i = CHANNEL * (y*DEPTH_WIDTH + x);

				grid[x][y] = true;

				// Set the RGB of this pixel to white
				buf2[i] = 255;
				buf2[i + 1] = 255;
				buf2[i + 2] = 255;
			}
		}
		if (targetDepthIndex != 0)
		{
			USHORT coordDepth = pBuff[targetDepthIndex];
			if (coordDepth == 0x0000) {
				coordDepth = 65000;
			}
			int gridY = ((int)coordDepth) / (65535 / DEPTH_HEIGHT);
			int gridIndex = CHANNEL * (gridY*DEPTH_WIDTH + targetDepthX);
			buf2[gridIndex] = 0;
			buf2[gridIndex + 1] = 0;
			buf2[gridIndex + 2] = 255;
			while (grid[targetDepthX][gridY] && gridY > 0) {
				gridY--;
			}
			targetVector = { 120, 0, targetDepthX, gridY, true};
			if (gridY == 0) {
				targetVector.update = false;
			}
		}
	}
	return targetVector;
}

struct Point {
	int x;
	int y;
};

bool pointEqual(struct Point p1, struct Point p2)
{
	return (p1.x == p2.x && p1.y == p2.y);
}

int pointDistance(struct Point p1, struct Point p2)
{
	return int(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)));
}

int weightVal(int x, int y)
{
	if (grid[x][y]) {
		return 1;
	}
	else {
		return 100000;
	}
}

// The set of nodes already evaluated.
bool closedSet[DEPTH_WIDTH][DEPTH_HEIGHT];
// 320 x 240 < 80000
struct Point openSet[80000];
// For each node, which node it can most efficiently be reached from.
// If a node can be reached from many nodes, cameFrom will eventually contain the
// most efficient previous step.
struct Point cameFrom[DEPTH_WIDTH][DEPTH_HEIGHT];
// For each node, the cost of getting from the start node to that node.
int gArray[DEPTH_WIDTH][DEPTH_HEIGHT];

struct Vector AStarVector(int x1, int y1, int x2, int y2)
{
	struct Vector moveVector;
	moveVector = { 0,0,0,0,false };
	// The psuedocode for this search was adopted from wikipedia.

	struct Point p1 = { x1, y1 };
	struct Point p2 = { x2, y2 };
	struct Point endStack = { -1, -1 };
	// The set of currently discovered nodes still to be evaluated.
	// Initially, only the start node is known.
	openSet[0] = p1;
	openSet[1] = endStack;
	int openEnd = 1;

	// For each node, the total cost of getting from the start node to the goal
	// by passing by that node. That value is partly known, partly heuristic.
	int fArray[DEPTH_WIDTH][DEPTH_HEIGHT];
	for (int i = 0; i < DEPTH_WIDTH; i++)
	{
		for (int j = 0; j < DEPTH_HEIGHT; j++)
		{
			fArray[i][j] = numeric_limits<int>::max();
			gArray[i][j] = numeric_limits<int>::max();
			closedSet[i][j] = false;
			cameFrom[i][j] = { -1,-1 };
		}
	}
	
	// The cost of going from start to start is zero.
	gArray[p1.x][p1.y] = 0;

	// For the first node, that value is completely heuristic.
	fArray[p1.x][p1.y] = pointDistance(p1, p2) + weightVal(p1.x, p1.y);

	struct Point secondPoint = { -1, -1 };

	int j = 0;
	while (openSet[0].x != endStack.x && j < 1000) {
		j++;
		struct Point current = openSet[0];
		int current_i = 0;
		for (int i = 0; i < openEnd; i++) 
		{
			struct Point p = openSet[i];
			if (fArray[p.x][p.y] < fArray[current.x][current.y]) {
				current = p;
				current_i = i;
			}
		}

		// Terminate if current is the end node
		if (current.y == p2.x && current.y == p2.y) 
		{
			break;
		}

		// Remove current from the open set
		for (int i = current_i; i < openEnd-1; i++) 
		{
			openSet[i] = openSet[i + 1];
		}
		openEnd--;
		
		closedSet[current.x][current.y] = true;
		for (int i = 0; i < aStarModsMax; i += 2) {
			int x = current.x + aStarMods[i];
			int y = current.y + aStarMods[i + 1];
			if (x < 0 || y < 0 || x >= DEPTH_WIDTH || y >= DEPTH_HEIGHT|| closedSet[x][y])
			{
				continue; // Ignore the neighbor
			}
			struct Point neighbor = { x, y };
			
			// The distance from start to a neighbor

			int tentative_gScore = gArray[current.x][current.y] + 1 + weightVal(x, y);
			bool inOpenSet = false;
			
			for (int i = 0; i < openEnd; i++)
			{
				struct Point p = openSet[i];
				if (p.x == x && p.y == y) {
					inOpenSet = true;
					break;
				}
			}
			if (!inOpenSet) {
				openSet[openEnd + 1] = endStack;
				openSet[openEnd] = neighbor;
				openEnd++;
			}
			else if (tentative_gScore >= gArray[x][y]){
				continue; // This is not a better path.
			}

			// This path is the best until now. Record it!
			if (pointEqual(current, p1)) {
				secondPoint = neighbor;
			}
			cameFrom[x][y] = current;
			gArray[x][y] = tentative_gScore;
			fArray[x][y] = gArray[x][y] + pointDistance(neighbor, p2);
		}
	}
	moveVector = { p1.x, p1.y, secondPoint.x, secondPoint.y, true };
	return moveVector;
}

void drawMovementVector(IplImage* gridImage, struct Vector movementVector)
{
	// Todo: could draw line from point 1 to point 2 on vector
	int i = CHANNEL * (movementVector.y1*DEPTH_WIDTH + movementVector.x1);
	int i2 = CHANNEL * (movementVector.y2*DEPTH_WIDTH + movementVector.x2);
	buf2[i] = 255;
	buf2[i + 1] = 0;
	buf2[i + 2] = 0;
	buf2[i2] = 255;
	buf2[i2 + 1] = 0;
	buf2[i2 + 2] = 0;
	cvSetData(gridImage, buf2, DEPTH_WIDTH * CHANNEL);
}

int main(int argc, char * argv[])
{
	IplImage* color = cvCreateImageHeader(cvSize(COLOR_WIDTH, COLOR_HEIGHT), IPL_DEPTH_8U, 4);
	IplImage* colorTest = cvCreateImageHeader(cvSize(COLOR_WIDTH, COLOR_HEIGHT), IPL_DEPTH_8U, CHANNEL);
	IplImage* depth = cvCreateImageHeader(cvSize(DEPTH_WIDTH, DEPTH_HEIGHT), IPL_DEPTH_8U, CHANNEL);
	IplImage* gridImage = cvCreateImageHeader(cvSize(DEPTH_WIDTH, DEPTH_HEIGHT), IPL_DEPTH_8U, CHANNEL);
	

	cvNamedWindow("color image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("color test", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("depth image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("grid image", CV_WINDOW_AUTOSIZE);

	HRESULT hr = NuiInitialize(
		NUI_INITIALIZE_FLAG_USES_DEPTH
		| NUI_INITIALIZE_FLAG_USES_COLOR);

	if (hr != S_OK)
	{
		cout << "NuiInitialize failed" << endl;
		cvWaitKey(0);
		return hr;
	}

	// Initialization of the Color stream
	HANDLE h1 = CreateEvent(NULL, TRUE, FALSE, NULL);
	HANDLE h2 = NULL;
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
		0, 2, h1, &h2);

	if (FAILED(hr))
	{
		cout << "Could not open image stream video" << endl;
		cvWaitKey(0);
		return hr;
	}

	// Initialization of the Depth stream
	HANDLE h3 = CreateEvent(NULL, TRUE, FALSE, NULL);
	HANDLE h4 = NULL;

	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH,
		NUI_IMAGE_RESOLUTION_320x240, 0, 1, h3, &h4);
	if (FAILED(hr))
	{
		std::cout << "Could not open depth stream video" << std::endl;
		cvWaitKey(0);
		return hr;
	}

	INuiFrameTexture * cTexture;
	INuiFrameTexture * dTexture;

	struct Vector targetVector = { 0,0,0,0,false };
	struct Vector movementVector = { 0,0,0,0,false };

	while (1)
	{	
		// Access the current Color Frame
		WaitForSingleObject(h1, INFINITE);
		const NUI_IMAGE_FRAME * cImageFrame = getFrame(h2);
		cTexture = cImageFrame->pFrameTexture;
		
		// For debug, show the current color frame
		//drawColor(cTexture, color);
		//cvShowImage("color image", color);
		
		// Access the current Depth frame
		WaitForSingleObject(h3, INFINITE);
		const NUI_IMAGE_FRAME * dImageFrame = getFrame(h4);
		dTexture = dImageFrame->pFrameTexture;
		
		// For debug, show the current depth frame
		//drawDepth(dTexture, depth);
		//cvShowImage("depth image", depth);

		// Show valid places to move		
		targetVector = drawMoveGrid(cTexture, dTexture, gridImage, colorTest);
		// If the target vector signifies that it is new
		if (targetVector.update) 
		{
			struct Vector movementVector2 = AStarVector(targetVector.x1, targetVector.y1, targetVector.x2, targetVector.y2);
			if (movementVector2.update) {
				movementVector = movementVector2;
			}
		}
		drawMovementVector(gridImage, movementVector);
		cvShowImage("grid image", gridImage);
		cvShowImage("color test", colorTest);

		// Release the color frame
		NuiImageStreamReleaseFrame(h2, cImageFrame);

		// Release the depth frame
		NuiImageStreamReleaseFrame(h4, dImageFrame);

		//exit
		int c = cvWaitKey(1);
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}

	cvReleaseImageHeader(&depth);
	cvReleaseImageHeader(&color);
	cvReleaseImageHeader(&gridImage);
	cvReleaseImageHeader(&colorTest);
	cvDestroyWindow("depth image");
	cvDestroyWindow("color image");
	cvDestroyWindow("grid image");
	cvDestroyWindow("color test");

	NuiShutdown();

	return 0;

}