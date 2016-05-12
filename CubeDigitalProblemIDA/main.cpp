#include "CubeIDA.h"
const int CUBENUM = 27; // the number of cube's grid
clock_t TimesStart, TimesEnd; // For measuring the time of the executing of the algorithm
Pool CubePool;
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
	// initialization and start IDA*
#ifdef H1_HEURISTIC
	source.heuristic = CalculateHeuristic(source, target);
#else
	source.heuristic = CalculateHeuristic(source, target, cubeMap);
#endif // H1_HEURISTIC
	Cube *pSource = new Cube(source);
	StartIDAStar(pSource, target, cubeMap);
	//getchar();
	return 0;
}