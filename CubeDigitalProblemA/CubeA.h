#ifndef _CUBEA_H_
#define _CUBEA_H_

#include <iostream>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include <queue>
#include <functional>
#include <set>
#include <stack>
#include <unordered_set>
#include <map>
#include <ctime>
using namespace std;

extern const int CUBENUM; // the number of cube's grid
extern clock_t TimesStart, TimesEnd; // For measuring the time of the executing of the algorithm
enum actions {
	BEGIN, U = 'U', D = 'D', L = 'L', R = 'R', F = 'F', B = 'B'
};
struct Cube {
	int heuristic;		// f()=g()+h()
	enum actions action;// the action of this step
	int step;			// steps from the begin
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
/* offer a comparison function for equality for the set<> */
struct CubeStarCmp {
	bool operator()(const Cube*const&lhs, const Cube*const&rhs) const { // 'const' is required
		return lhs->Matrix < rhs->Matrix;
	}
};
/* a set of Cube* , for delete the repetition of Cube* */
// set<Cube*, CubeStarCmp> SetOfCube;
/* offer a equality function for unordered_set<> */
struct CubeStarEqual {
	bool operator()(const Cube*const&lhs, const Cube*const&rhs) const { // 'const' is required
		return (lhs->Matrix == rhs->Matrix);
	}
};
/* offer a hash function for unordered_set<>
* bkdr algorithm --- a sample hash function of string
*/
struct KeyHasher {
	size_t operator()(const Cube*const&cube) const {
		register size_t hash = 0;
		for (size_t i = 0; i < (unsigned)CUBENUM; ++i) {
			int ch = cube->Matrix[i];
			if (ch >= 0)
				hash = hash * 131 + (size_t)ch;
			// 也可以乘以31、131、1313、13131、131313..  
			// 有人说将乘法分解为位运算及加减法可以提高效率，如将上式表达为：hash = hash << 7 + hash << 1 + hash + ch;  
			// 但其实在Intel平台上，CPU内部对二者的处理效率都是差不多的，  
			// 我分别进行了100亿次的上述两种运算，发现二者时间差距基本为0（如果是Debug版，分解成位运算后的耗时还要高1/3）；  
			// 在ARM这类RISC系统上没有测试过，由于ARM内部使用Booth's Algorithm来模拟32位整数乘法运算，它的效率与乘数有关：  
			// 当乘数8-31位都为1或0时，需要1个时钟周期  
			// 当乘数16-31位都为1或0时，需要2个时钟周期  
			// 当乘数24-31位都为1或0时，需要3个时钟周期  
			// 否则，需要4个时钟周期  
			// 因此，虽然我没有实际测试，但是我依然认为二者效率上差别不大  
		}
		return hash;
	}
};
extern const size_t UnorderedSetSize;
extern unordered_set<Cube*, KeyHasher, CubeStarEqual> SetOfCubeOpen;
extern unordered_set<Cube*, KeyHasher, CubeStarEqual> SetOfCubeClose;
/* offer a comparison function for equality for the priority_queue<> */
struct CubeCmp {
	bool operator()(Cube*const &lhs, Cube*const &rhs) {
		if (lhs->heuristic == rhs->heuristic)
			return ((lhs->heuristic - lhs->step) > (rhs->heuristic - rhs->step));
		return lhs->heuristic > rhs->heuristic;
	}
};

/* convert a number to <x, y, z> vector */
void ConvertToVector(const int index, int&x, int&y, int&z);
/* convert a <x, y, z> vector to number */
void ConvertToNumber(int&index, const int x, const int y, const int z);
/* h1 heuristic function , calculate the number of difference of each grid */
int CalculateHeuristic(Cube c, Cube target);
/* h2 heuristic function , calculate the direct distance of each grid */
int CalculateHeuristic(Cube c, Cube target, const int cubeMap[]);
void TreatState(enum actions action, int pos, Cube*const pCurrent, const Cube target, priority_queue<Cube*, vector<Cube*>, CubeCmp>&cubeQueue, const int cubeMap[], int x, int y, int z);
/* Add state into cubeQueue according to the current state and target state
* at most 6 choices
*/
void AddState(Cube*const pCurrent, Cube target, priority_queue<Cube*, vector<Cube*>, CubeCmp>&cubeQueue, const int cubeMap[]);
/* print the output file. Including the execute time, steps, path */
void DumpFile(string filename, Cube*pC);
/* Start A* algorithm
* each time pop a privorty_queue<>
*/
void StartAstar(Cube*const pSource, Cube target, const int cubeMap[]);
/* function for file input */
void TreatFile(string fileName, Cube&cube);

#endif

