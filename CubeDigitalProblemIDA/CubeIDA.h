#ifndef _CUBEIDA_H_
#define _CUBEIDA_H_

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <stack>
#include <algorithm>
#include <ctime>
#include <cassert>
// #define H1_HEURISTIC
using namespace std;

extern const int CUBENUM; // the number of cube's grid
extern clock_t TimesStart, TimesEnd; // For measuring the time of the executing of the algorithm

enum actions {
	BEGIN, U = 'U', D = 'D', L = 'L', R = 'R', F = 'F', B = 'B'
};
struct Cube {
	int heuristic;				// f()=g()+h()
	int step;					// steps from the begin
	enum actions action;		// the action of this step
	struct Cube*previous;// the Cube of previous one
						 /* (0,0)--> 0  (0,1)--> 1  (0,2)--> 2
						 * (1,0)--> 3  (1,1)--> 4  (1,2)--> 5
						 * (2,0)--> 6  ...  */
	string Matrix;		// Cube Matrix
						// constructor
	Cube() :heuristic(0), action(BEGIN), step(0), previous(0), Matrix(CUBENUM, 0) {};
	// destructor
	~Cube() {};
};
class Pool {
public:	Cube*pool;
		vector<Cube*> vPool;
private:const unsigned int LENGTH = 1000000;	// the length of each poo;
		unsigned int indexPool, indexVector;	// index of poll, vector
public:
	/* constructor */
	Pool() :indexPool(0), indexVector(0) {
		pool = new Cube[LENGTH];
		vPool.push_back(pool);
	}
	void clear();
	Cube*addState(const Cube*const current);
	void decrease();
	~Pool() {
		for (vector<Cube*>::iterator it = vPool.begin(); it != vPool.end(); ++it)
			delete[](*it);
	}
};
extern Pool CubePool;

/* convert a number to <x, y, z> vector */
void ConvertToVector(const int index, int&x, int&y, int&z);
/* convert a <x, y, z> vector to number */
void ConvertToNumber(int&index, const int x, const int y, const int z);
/* h1 heuristic function , calculate the number of difference of each grid */
int CalculateHeuristic(Cube c, Cube target);
/* h2 heuristic function , calculate the direct distance of each grid */
int CalculateHeuristic(Cube c, Cube target, const int cubeMap[]);
void TreatState(enum actions action, int pos, Cube*const pCurrent, const Cube target, stack<Cube*>&stackCube, const int cubeMap[], int x, int y, int z);
/* Add state into stackCube according to the current state and target state
* at most 6 choices
*/
void AddState(Cube*const pCurrent, const Cube target, stack<Cube*>&stackCube, const int cubeMap[]);
/* print the output file. Including the execute time, steps, path */
void DumpFile(string filename, Cube*pC);
void StartIDAStar(Cube*source, Cube target, const int cubeMap[]);
/* function for file input */
void TreatFile(string fileName, Cube&cube);

#endif // !_CUBEIDA_H_