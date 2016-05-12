#include "CubeA.h"
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
// #define H1_HEURISTIC
using namespace std;

const int CUBENUM = 27; // the number of cube's grid
clock_t TimesStart, TimesEnd; // For measuring the time of the executing of the algorithm
const size_t UnorderedSetSize = size_t(11200000 * 1.75);
unordered_set<Cube*, KeyHasher, CubeStarEqual> SetOfCubeOpen(UnorderedSetSize);
unordered_set<Cube*, KeyHasher, CubeStarEqual> SetOfCubeClose(UnorderedSetSize);
int main()
{
	Cube source;
	TreatFile("source.txt", source);
	Cube target;
	TreatFile("target.txt", target);
	// treasure the source data. because it costs a lot of time after analyze.
	int cubeMap[26] = { 0 };
	for (int i = 0; i < CUBENUM; ++i)
		if (target.Matrix[i] > 0)
			cubeMap[target.Matrix[i]] = i;
	// initialization and start A*
#ifdef H1_HEURISTIC
	source.heuristic = CalculateHeuristic(source, target);
#else
	source.heuristic = CalculateHeuristic(source, target, cubeMap);
#endif // H1_HEURISTIC
	Cube *pSource = new Cube(source);
	StartAstar(pSource, target, cubeMap);
	// getchar();
	return 0;
}