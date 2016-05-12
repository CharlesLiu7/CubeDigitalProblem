#include "CubeA.h"

/* convert a number to <x, y, z> vector */
void ConvertToVector(const int index, int&x, int&y, int&z) {
	if (index > 26 || index < 0) {
		cout << "out of index" << endl;
		exit(-1);
	}
	int temp = index;
	z = temp % 3;
	temp /= 3;
	y = temp % 3;
	temp /= 3;
	x = temp % 3;
}
/* convert a <x, y, z> vector to number */
void ConvertToNumber(int&index, const int x, const int y, const int z) {
	index = x * 3 * 3 + y * 3 + z;
}
/* h1 heuristic function , calculate the number of difference of each grid */
int CalculateHeuristic(Cube c, Cube target) {
	int heristic = 0;
	for (int i = 0; i < CUBENUM; ++i)
		if (c.Matrix[i]>0 && target.Matrix[i] != c.Matrix[i])
			heristic++;
	return heristic;
}
/* h2 heuristic function , calculate the direct distance of each grid */
int CalculateHeuristic(Cube c, Cube target, const int cubeMap[]) {
	int herisitic = 0;
	int x, y, z;
	int r, s, t;
	for (int i = 0; i < CUBENUM; ++i) {
		if (c.Matrix[i] > 0 && c.Matrix[i] != target.Matrix[i]) {
			ConvertToVector(i, x, y, z);
			ConvertToVector(cubeMap[c.Matrix[i]], r, s, t);
			herisitic += (abs(x - r) + abs(y - s) + abs(z - t));
		}
	}
	return herisitic;
}
void TreatState(enum actions action, int pos, Cube*const pCurrent, const Cube target, priority_queue<Cube*, vector<Cube*>, CubeCmp>&cubeQueue, const int cubeMap[], int x, int y, int z) {
	int posNext;
	ConvertToNumber(posNext, x, y, z);
	if (pCurrent->Matrix[posNext] != -1) {
		Cube*temp = new Cube(*pCurrent);
		swap(temp->Matrix[pos], temp->Matrix[posNext]);
		// remove the repetition of the state in SetOfCube
		if (!SetOfCubeClose.count(temp)) {
			temp->step = pCurrent->step + 1;
			temp->action = action;
#ifdef H1_HEURISTIC
			temp->heuristic = temp->step + CalculateHeuristic((*temp), target);
#else
			temp->heuristic = temp->step + CalculateHeuristic((*temp), target, cubeMap);
#endif // H1_HEURISTIC	
			Cube*re = *SetOfCubeOpen.find(temp);
			if (re != (*SetOfCubeOpen.end()) && re->heuristic > temp->heuristic) {
				re->step = temp->step;
				re->action = temp->action;
				re->heuristic = temp->heuristic;
				re->previous = pCurrent;
				// cubeQueue.push(re);
				delete temp;
			}
			else {
				temp->previous = pCurrent;
				SetOfCubeOpen.insert(temp);
				cubeQueue.push(temp);
			}
		}
		else {
			delete temp;
		}
	}
}
/* Add state into cubeQueue according to the current state and target state
* at most 6 choices
*/
void AddState(Cube*const pCurrent, Cube target, priority_queue<Cube*, vector<Cube*>, CubeCmp>&cubeQueue, const int cubeMap[]) {
	int x, y, z;
	string::size_type pos = pCurrent->Matrix.find_first_of((char)0);
	ConvertToVector(pos, x, y, z);
	if (x - 1 >= 0)
		TreatState(F, pos, pCurrent, target, cubeQueue, cubeMap, x - 1, y, z);
	if (x + 1 <= 2)
		TreatState(B, pos, pCurrent, target, cubeQueue, cubeMap, x + 1, y, z);
	if (y - 1 >= 0)
		TreatState(U, pos, pCurrent, target, cubeQueue, cubeMap, x, y - 1, z);
	if (y + 1 <= 2)
		TreatState(D, pos, pCurrent, target, cubeQueue, cubeMap, x, y + 1, z);
	if (z - 1 >= 0)
		TreatState(L, pos, pCurrent, target, cubeQueue, cubeMap, x, y, z - 1);
	if (z + 1 <= 2)
		TreatState(R, pos, pCurrent, target, cubeQueue, cubeMap, x, y, z + 1);
}
/* print the output file. Including the execute time, steps, path */
void DumpFile(string filename, Cube*pC) {
	ofstream fm(filename, ios::out);
	fm << ((double)(TimesEnd - TimesStart) / CLOCKS_PER_SEC) << "s" << endl;
	fm << "steps:" << pC->step << endl;
	Cube*p = pC;
	stack<char> actionPath;
	while (p->previous) {
		actionPath.push(p->action);
		p = p->previous;
	}
	while (!actionPath.empty()) {
		fm << actionPath.top();
		actionPath.pop();
	}
}
/* Start A* algorithm
* each time pop a privorty_queue<>
*/
void StartAstar(Cube*const pSource, Cube target, const int cubeMap[]) {
	priority_queue<Cube*, vector<Cube*>, CubeCmp> cubeQueue;
	cubeQueue.push(pSource);
	Cube*pc = cubeQueue.top();
	/*
	1 把它从开启列表中删除，然后添加到关闭列表中。
	2 检查所有相邻格子。跳过那些已经在关闭列表中的或者不可通过的(有墙，水的地形，或者其他无法通过的地形)，
	把他们添加进开启列表，如果他们还不在里面的话。把选中的方格作为新的方格的父节点。
	3 如果某个相邻格已经在开启列表里了，检查现在的这条路径是否更好。换句话说，检查如果我们用新的路径到达
	它的话，G值是否会更低一些。如果不是，那就什么都不做。
	*/
	SetOfCubeClose.insert(pc);
	int distance = pc->heuristic;
	TimesStart = clock(); // start to count time
	while (distance != 0) {
		cubeQueue.pop();
		AddState(pc, target, cubeQueue, cubeMap);
		pc = cubeQueue.top();
		SetOfCubeOpen.erase(pc);
		SetOfCubeClose.insert(pc);
#ifdef PRINT_DETAIL
		cout << char((*pc).action) << " " << "heuristic:" << (*c).heuristic << " step:" << (*c).step << endl;
#endif
		distance = CalculateHeuristic(*pc, target);
	}
	TimesEnd = clock(); // end to count time
						// print the action path of the result
	DumpFile("output.txt", pc);
	// for release the memory malloced by `new`
	for (unordered_set<Cube*, KeyHasher, CubeStarEqual>::iterator it = SetOfCubeOpen.begin(); it != SetOfCubeOpen.end(); ++it)
		delete (*it);
	for (unordered_set<Cube*, KeyHasher, CubeStarEqual>::iterator it = SetOfCubeClose.begin(); it != SetOfCubeClose.end(); ++it)
		delete (*it);
}
/* function for file input */
void TreatFile(string fileName, Cube&cube) {
	ifstream fm(fileName, ios::in);
	if (!fm) {
		cout << "fail to open " << fileName << endl;
		exit(-1);
	}
	for (int i = 0; i < CUBENUM; ++i) {
		int j;
		fm >> j;
		cube.Matrix[i] = j;
	}
	fm.close();
}