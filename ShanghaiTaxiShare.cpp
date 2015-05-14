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
#define MAX_PAIRS 400
#define SPEED     40          // 车速 

typedef float DistVector [435288];

float minDist[ALL_NODES], antiMinDist[ALL_NODES];
bool  visit[ALL_NODES];
queue<int> q;
vector<int> path[MAX_PAIRS];

// 图中的节点 
struct RoadNode {
	int blockNum, blockOffset, borderNum, blockBorderNum;
	float lat, lon;
	vector<float> distToBorder;
	vector<int> neighbourBorderNum;
	vector<float> neighbourBorderDistance;
};

// 请求的数据结构 
struct QueryInfo {
	int startTd, endId;
	vector<float> distS, distT;
	int latestTime;
	int maxRatio;
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


/*
 *生成同起点情况下的测试数据，起点坐标：(31.1622,121.54)
 *数据格式：终点Id，所能接受的省钱系数 
 */
void generateTestsForCase1(){
	SPFA(31948);
	
	srand((unsigned)time(0));
	int positionId[MAX_PAIRS];
	float ratio[MAX_PAIRS];
	
	ofstream fout("test1.txt");
	for (int i = 0; i < MAX_PAIRS; i++) {
		int s = rand() % ALL_NODES;
		while (minDist[s] == MAX_DIST) {
			s = rand() % ALL_NODES;
		} 
		int time  = rand() % 60;
		float rat = rand() % 30 + 50; 
		positionId[i] = s;
		ratio[i]      = rat / 100; 
	}
	
	for (int i = 0; i < MAX_PAIRS; i++) {
		fout<<positionId[i]<<" "<<ratio[i]<<endl;
	}
	
	fout.close();
	
	
}

/*
 * 生成同终点情况下的测试数据，终点坐标：浦东国际机场 
 * 数据格式：起点Id，最晚出发时间，所能接受的省钱系数 
 */
void generateTestsForCase2() {
	antiSPFA(411445);
	srand((unsigned)time(0));
	int positionId[MAX_PAIRS], latestTime[MAX_PAIRS];
	float ratio[MAX_PAIRS];
	
	ofstream fout("test2.txt");
	for (int i = 0; i < MAX_PAIRS; i++) {
		int s = rand() % ALL_NODES;
		while (antiMinDist[s] == MAX_DIST) {
			s = rand() % ALL_NODES;
		} 
		int time  = rand() % 60;
		float rat = rand() % 30 + 50; 
		positionId[i] = s;
		latestTime[i] = time;
		ratio[i]      = rat / 100; 
	}
	
	for (int i = 0; i < MAX_PAIRS; i++) {
		fout<<positionId[i]<<" "<<latestTime[i]<<" "<<ratio[i]<<endl;
	}
	
	fout.close();
}

/*
 * 起点和终点都不相同的情况
 * 数据格式：起点id，终点id，最晚出发时间，省钱比例 
 */
void generateTestsForCase3() {
	ifstream fin("testcase.txt");
	int s, t;
	int startId[2000], endId[2000], lastestTime[2000];
	float ratio[2000];
	int tot = 0;
	srand((unsigned)time(0));
	while (fin>>s>>t) 
	{
		startId[tot] = s;
		endId[tot]   = t;
		int time  = rand() % 60;
		float rat = rand() % 30 + 50; 
		lastestTime[tot] = time;
		ratio[tot++]     = rat / 100; 
	}
	fin.close();
	
	ofstream fout("test3.txt");
	for (int i = 0; i < tot; i++) {
		fout<<startId[i]<<" "<<endId[i]<<" "<<lastestTime[i]<<" "<<ratio[i]<<endl;
	}
	fout.close();
	
	
}

float getSaveRatio(float dist1, float dist2, float totDist) {
	float saveAll = (dist1 + dist2 - totDist) * 0.8;
	
	return 1 - (0.8 * saveAll) / (dist1 + dist2);
}

float shareDistance(DistVector distS1, DistVector distT1, DistVector distS2, DistVector distT2, int s1, int t1, int s2, int t2, int lastTime1, int lastTime2) {
	float ans = MAX_DIST;
	//s1->s2->t1->t2
	if (distS1[s2] + distS2[t1] + distT1[t2] < ans && distS1[s2] / SPEED < lastTime2) {
		ans = distS1[s2] + distS2[t1] + distT1[t2];
	}
	//s1->s2->t2->t1
	if (distS1[s2] + distS2[t2] + distT2[t1] < ans && distS1[s2] / SPEED < lastTime2) {
		ans = distS1[s2] + distS2[t2] + distT2[t1];
	}
	//s2->s1->t1->t2
	if (distS2[s1] + distS1[t1] + distT1[t2] < ans && distS2[s1] / SPEED < lastTime1) {
		ans = distS2[s1] + distS1[t1] + distT1[t2];
	}
	//s2->s1->t2->t1
	if (distS2[s1] + distS1[t2] + distT2[t1] < ans && distS2[s1] / SPEED < lastTime1) {
		ans = distS2[s1] + distS1[t2] + distT2[t1];
	}
	
	return ans;
	
}




int main() {
	init();

	time_t rawtime; 
	struct tm * timeinfo; 

	time(&rawtime); 
	timeinfo = localtime(&rawtime); 
	printf ("系统时间是: %s", asctime (timeinfo) ); 
	
	generateTestsForCase1();
    generateTestsForCase2();
	generateTestsForCase3();

	time(&rawtime); 
	timeinfo = localtime(&rawtime); 
	printf ("系统时间是: %s", asctime (timeinfo) );
	
	//recycle();

	return 0;
}






