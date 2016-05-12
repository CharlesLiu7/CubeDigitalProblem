#include "CubeIDA.h"

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
void TreatState(enum actions action, int pos, Cube*const pCurrent, const Cube target, stack<Cube*>&stackCube, const int cubeMap[], int x, int y, int z) {
	int posNext;
	ConvertToNumber(posNext, x, y, z);
	if (pCurrent->Matrix[posNext] != -1) {
		Cube*temp = CubePool.addState(pCurrent);
		swap(temp->Matrix[pos], temp->Matrix[posNext]);
		// remove the state if the current status is the father of the pCurrent
		if (pCurrent->previous == NULL || temp->Matrix != pCurrent->previous->Matrix) {
			temp->step = pCurrent->step + 1;
			temp->action = action;
#ifdef H1_HEURISTIC
			temp->heuristic = temp->step + CalculateHeuristic((*temp), target);
#else
			temp->heuristic = temp->step + CalculateHeuristic((*temp), target, cubeMap);
#endif // H1_HEURISTIC		
			temp->previous = pCurrent;
			stackCube.push(temp);
		}
		else {
			CubePool.decrease();
		}
	}
}
/* Add state into stackCube according to the current state and target state
* at most 6 choices
*/
void AddState(Cube*const pCurrent, const Cube target, stack<Cube*>&stackCube, const int cubeMap[]) {
	int x, y, z;
	string::size_type pos = pCurrent->Matrix.find_first_of((char)0);
	ConvertToVector(pos, x, y, z);
	if (x - 1 >= 0)
		TreatState(F, pos, pCurrent, target, stackCube, cubeMap, x - 1, y, z);
	if (x + 1 <= 2)
		TreatState(B, pos, pCurrent, target, stackCube, cubeMap, x + 1, y, z);
	if (y - 1 >= 0)
		TreatState(U, pos, pCurrent, target, stackCube, cubeMap, x, y - 1, z);
	if (y + 1 <= 2)
		TreatState(D, pos, pCurrent, target, stackCube, cubeMap, x, y + 1, z);
	if (z - 1 >= 0)
		TreatState(L, pos, pCurrent, target, stackCube, cubeMap, x, y, z - 1);
	if (z + 1 <= 2)
		TreatState(R, pos, pCurrent, target, stackCube, cubeMap, x, y, z + 1);
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
/* Start IDA* algorithm
* pseudo-code
* d_limit <-- d(s0)
* while d_limit<MAX do
*	next_d_limit <-- MAX
*	list <-- { s0 }
*	while list is not empty do
*		s <-- head(list)
*		list <-- rest(list)
*		if d(s)>d_limit then
*			next_d_limit <-- min(next_d_limit, d(s))
*		else
*			if s is a goal then
*				return s
*			end if
*			newstates <-- apply action to s
*			list <-- prepend(newstate, list)
*		end if
*	end while
*	d_limit=next_d_limit
* end while
*/
void StartIDAStar(Cube*source, Cube target, const int cubeMap[]) {
	const int MAX = 65535;
	int depthLimit = source->heuristic;
	stack<Cube*> stackCube;
	TimesStart = clock();
	while (true) {
		int nextDepthLimit = MAX;
		stackCube.push(source);
		CubePool.clear();
		while (!stackCube.empty()) {
			Cube*current = stackCube.top();
			stackCube.pop();
			if (current->heuristic > depthLimit)
				nextDepthLimit = min(nextDepthLimit, current->heuristic);
			else {
				if (CalculateHeuristic(*current, target) == 0) {
					TimesEnd = clock();
					DumpFile("output.txt", current);
					return;
				}
				AddState(current, target, stackCube, cubeMap);
			}
		}
		depthLimit = nextDepthLimit;
	}
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

void Pool::clear() {
	indexVector = indexPool = 0;
	pool = (*vPool.begin());
}
Cube* Pool::addState(const Cube*const current) {
	if (indexPool < LENGTH - 1) {
		pool[indexPool] = (*current);
		return &pool[indexPool++];
	}
	else if (indexPool == (LENGTH - 1))
		if (indexVector < (vPool.size() - 1)) {
			pool[indexPool] = (*current);
			++indexVector;
			pool = (*(vPool.begin() + indexVector));
			indexPool = 0;
			return &((*(vPool.begin() + indexVector - 1))[LENGTH - 1]);
		}
		else if (indexVector == (vPool.size() - 1)) {
			pool[indexPool] = (*current);
			++indexVector;
			pool = new Cube[LENGTH];
			indexPool = 0;
			vPool.push_back(pool);
			return &((*(vPool.begin() + indexVector - 1))[LENGTH - 1]);
		}
		else
			assert(" Pool add state error 1 ");
	else
		assert(" Pool add state error 2 ");
	assert(" Pool add state error 3 ");
	return 0;
}
void Pool::decrease() {
	if (indexPool == 0) {
		indexPool = LENGTH - 1;
		--indexVector;
		pool = (*(vPool.begin() + indexVector));
	}
	else
		--indexPool;
}
