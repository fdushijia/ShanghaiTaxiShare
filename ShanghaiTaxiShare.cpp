#include <iostream>
#include <string>
#include <list>
#include <fstream>
#include <vector>
#include <cstring>
#include <queue>
#include <stdlib.h> 
#include <time.h>
#include "Distance.h"
#include <malloc.h>

using namespace std;

#define ALL_NODES 435288      //图中Node的最大值


#define MAX_DIST  1000000000  //初始最大距离
#define MAX_PAIRS 200

float minDist[ALL_NODES], antiMinDist[ALL_NODES];
bool  visit[ALL_NODES];
queue<int> q;
vector<int> path[MAX_PAIRS];

struct RoadNode {
	int blockNum, blockOffset, borderNum, blockBorderNum;
	float lat, lon;
	vector<float> distToBorder;
	vector<int> neighbourBorderNum;
	vector<float> neighbourBorderDistance;
};

struct NeighNode {
	int id;
	float dist;
	NeighNode* next;
};
NeighNode* neighbour[ALL_NODES]; //正向边
NeighNode* fromNode[ALL_NODES];  //反向边

NeighNode *neighbourTail[ALL_NODES], *fromTail[ALL_NODES];
RoadNode roadNodes[ALL_NODES];
int fa[ALL_NODES], antiFa[ALL_NODES];
vector<float> distTemp[ALL_NODES];

void init() {

	cout<<"Constructing the road..."<<endl;
	ifstream fin("Nodes_with_block_infos.txt");
	string idTemp, latTemp, lonTemp, blockNumTemp, blockOffsetTemp, borderNumTemp, blockBorderNumTemp;
	int i = 0;

	cout<<"Init..."<<endl;
	while (fin>>idTemp>>latTemp>>lonTemp>>blockNumTemp>>blockOffsetTemp>>borderNumTemp>>blockBorderNumTemp) {
		roadNodes[i].lat = atof(latTemp.c_str());
		roadNodes[i].lon = atof(lonTemp.c_str());
		neighbour[i] = NULL;
		neighbourTail[i] = NULL;
		fromNode[i] = NULL;
		fromTail[i] = NULL;
		i++;
	}
	fin.close();
	
	cout<<"Constructing..."<<endl;
	ifstream wayIn("newWays_long.txt");
	string ss, tt;
	int s, t;
	while (wayIn>>ss>>tt) {
		s = atoi(ss.c_str());
		t = atoi(tt.c_str());

		// 建正向边
		if (neighbour[s] == NULL) {
			neighbour[s]       = (struct NeighNode *)malloc(sizeof(NeighNode));
			neighbour[s]->id   = t;
			neighbour[s]->dist = Distance::getDist(roadNodes[s].lat, roadNodes[s].lon, roadNodes[t].lat, roadNodes[t].lon);
			neighbour[s]->next = NULL;
			neighbourTail[s]   = neighbour[s];
		} else {
			NeighNode *p  = (struct NeighNode*)malloc(sizeof(NeighNode)); 
			p->id         = t;
			p->dist       = Distance::getDist(roadNodes[s].lat, roadNodes[s].lon, roadNodes[t].lat, roadNodes[t].lon);
			p->next       = NULL;
			neighbourTail[s]->next = p;
			neighbourTail[s]       = p;
		}

		// 建反向边
		if (fromNode[t] == NULL) {
			fromNode[t]       = (struct NeighNode*) malloc(sizeof(NeighNode));
			fromNode[t]->id   = s;
			fromNode[t]->dist = Distance::getDist(roadNodes[s].lat, roadNodes[s].lon, roadNodes[t].lat, roadNodes[t].lon);
			fromNode[t]->next = NULL;
			fromTail[t]       = fromNode[t];
		} else {
			NeighNode *p      = (struct NeighNode*)malloc(sizeof(NeighNode));
			p->id             = s;
			p->next           = NULL;
			p->dist           = Distance::getDist(roadNodes[s].lat, roadNodes[s].lon, roadNodes[t].lat, roadNodes[t].lon);
			fromTail[t]->next = p;
			fromTail[t]       = p;
		}
	}
	wayIn.close();	

}

void recycle() {
	cout<<"End..."<<endl;
	for (int i = 0; i < ALL_NODES; i++) {
		NeighNode* p = neighbour[i];
		NeighNode *q;
		if (p == NULL) continue;
		while (p != NULL) {
			q = p;
			p = p->next;
			delete q;
		}

		p = fromNode[i];
		while (p != NULL) {
			q = p;
			p = p->next;
			delete q;
		}

	}

	
}

void initDist(int s) {
	q.empty();
	for (int i = 0; i < ALL_NODES; i++) {
		minDist[i] = MAX_DIST;
		visit[i]   = false;
        fa[i] = 0;
	}
	minDist[s] = 0;
    fa[s] = -1;
}

void initAntiDist(int s) {
	q.empty();
	for (int i = 0; i < ALL_NODES; i++) {
		antiMinDist[i] = MAX_DIST;
		visit[i]       = false;
		fa[i]          = 0;
	}
	antiMinDist[s] = 0;
	antiFa[s] = -1;
}

/**
 * 因为路网转化成图之后是稀疏图，这里用SPFA算法求单源点最短路径
 */
void SPFA(int s) {
	initDist(s);
	q.push(s);
	int k;
	while (!q.empty()) {
		k = q.front();
		q.pop();
		visit[k] = false;
		NeighNode *p = neighbour[k];
		while (p) {
			int neighbourId = p->id;
			if (minDist[k] + p->dist < minDist[neighbourId]) {
				minDist[neighbourId] = minDist[k] + p->dist;
				fa[neighbourId] = k;
				if (!visit[neighbourId]) {
					visit[neighbourId] = true;
					q.push(neighbourId);
				}
			}
			p = p->next;

		}

	}
	for (int j = 0; j < ALL_NODES; j++)
        distTemp[s].push_back(minDist[j]);
}

// 单源点反向最短路径
void antiSPFA(int s) {
	initAntiDist(s);
	q.push(s);
	int k;
	while (!q.empty()) {
		k = q.front();
		q.pop();
		visit[k] = false;
		NeighNode *p = fromNode[k];
		while (p) {
			int fromId = p->id;
			if (antiMinDist[k] + p->dist < antiMinDist[fromId]) {
				antiMinDist[fromId] = antiMinDist[k] + p->dist;
				antiFa[fromId] = k;
				if (!visit[fromId]) {
					visit[fromId] = true;
					q.push(fromId);
				}
			}
			p = p->next;
		}
	}
}


/*vector<int> calcPath(int s, int t) {
	int i = t;
	vector<int> temp;
	bool used[BLOCK_NUM];
	for (int i = 0; i < BLOCK_NUM; i++) {
		used[i] = false;
	}
	while (i != -1) {
		if (!used[roadNodes[i].blockNum]) {
			temp.push_back(roadNodes[i].blockNum);
			used[roadNodes[i].blockNum] = true;
		}
		i = fa[i];
	}

	if (!used[roadNodes[s].blockNum])
		temp.push_back(roadNodes[s].blockNum);
	

    return temp;
}

void testShare() {
	int s, t;
	ifstream testIn("testdata1_ok.txt");
	int startId[MAX_PAIRS], endId[MAX_PAIRS];
	int pairs = 0;
	while (testIn>>s>>t) {
		startId[pairs] = s;
		endId[pairs]   = t;
		pairs ++;
	}
	testIn.close();

	ofstream distOut("PairsDistance.txt");
	for (int i = 0; i < pairs; i ++) {
		s = startId[i];
        t = endId[i];
        cout<<s<<" "<<t<<endl;
		SPFA(s);
		if (minDist[t] == MAX_DIST) {
			distOut<<"NO WAY!"<<endl;
		} else {
		    distOut<<minDist[t]<<endl;
		    if (distTemp[t].size() == 0) {
			    SPFA(t);
		    }
        }
	}
	distOut.close();	
	

	ofstream fout("Share_Pairs.txt");
	for (int i = 0; i < pairs; i++)
		for (int j = 0; j < pairs; j++) 
        if (i != j){
		//	cout<<i<<" "<<j<<endl;
			int s1 = startId[i];
			int t1 = endId[i];
			int s2 = startId[j];
			int t2 = endId[j];
			float ans = distTemp[s1][t1] + distTemp[s2][t2];
			float origin = ans;
			//s1->s2->t1->t2
			if (distTemp[s1][s2] + distTemp[s2][t1] + distTemp[t1][t2] < ans)
				ans = distTemp[s1][s2] + distTemp[s2][t1] + distTemp[t1][t2];
			//s1->s2->t2->t2
			if (distTemp[s1][s2] + distTemp[s2][t2] + distTemp[t2][t1] < ans)
				ans = distTemp[s1][s2] + distTemp[s2][t2] + distTemp[t2][t1];
			//s2->s1->t1->t2
			if (distTemp[s2][s1] + distTemp[s1][t1] + distTemp[t1][t2] < ans)
				ans = distTemp[s2][s1] + distTemp[s1][t1] + distTemp[t1][t2];
			//s2->s1->t2->t1
			if (distTemp[s2][s1] + distTemp[s1][t2] + distTemp[t2][t1] < ans)
				ans = distTemp[s2][s1] + distTemp[s1][t2] + distTemp[t2][t1];
            if (ans < origin) {
                fout<<i<<" "<<j<<endl;
			    fout<<"origin: "<<origin<<endl;
			    fout<<"ans   : "<<ans<<endl;
                fout<<ans - origin<<endl;
            }
		}

		fout.close();
	
} */

int main() {
	init();

	time_t rawtime; 
	struct tm * timeinfo; 

	time(&rawtime); 
	timeinfo = localtime(&rawtime); 
	printf ("系统时间是: %s", asctime (timeinfo) ); 
	
	antiSPFA(31948);

	ofstream fout("Pudong_Anti.txt");
	for (int i = 0; i < ALL_NODES; i++) {
		fout<<antiMinDist[i]<<" ";
	}

	fout<<endl;
	fout.close();

	time(&rawtime); 
	timeinfo = localtime(&rawtime); 
	printf ("系统时间是: %s", asctime (timeinfo) );
	
	//recycle();

	return 0;
}






