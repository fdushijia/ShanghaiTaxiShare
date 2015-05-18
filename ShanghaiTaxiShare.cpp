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


#define MAX_DIST   1000000000  //初始最大距离
#define MAX_PAIRS  400
#define TEST_PAIRS 200
#define SPEED      40          // 车速 
#define MAX_TIME   1000000000


float minDist[ALL_NODES], antiMinDist[ALL_NODES];
bool  visit[ALL_NODES];
queue<int> q;
vector<int> path[MAX_PAIRS];
int spouse[MAX_PAIRS];
int map[MAX_PAIRS][MAX_PAIRS];
vector<int> edge[MAX_PAIRS];

int Q[MAX_PAIRS],bot;
int mark[MAX_PAIRS];
int visited[MAX_PAIRS];
int belong[MAX_PAIRS];
int Next[MAX_PAIRS];


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
	int startId, endId;
	vector<float> distS, distT;
	int latestTime;
	float maxRatio;
	int queryId;
	time_t queryTime;
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
	for (int i = 0; i < 399; i++)
	{
		fin>>s>>t;
		startId[i] = s;
		endId[i]   = t;
		int time  = rand() % 60;
		float rat = rand() % 40 + 50; 
		lastestTime[i] = time;
		ratio[i]     = rat / 100; 
	}
	fin.close();

	ofstream fout("test3.txt");
	for (int i = 0; i < 399; i++) {
		fout<<startId[i]<<" "<<endId[i]<<" "<<lastestTime[i]<<" "<<ratio[i]<<endl;
	}
	fout.close();


}

float getSaveRatio(float dist1, float dist2, float totDist) {
	float saveAll = (dist1 + dist2 - totDist);

	return 1 - (0.95 * saveAll) / (dist1 + dist2);
}

float shareDistance(vector<float> distS1, vector<float> distT1, vector<float> distS2, vector<float> distT2, int s1, int t1, int s2, int t2, float lastTime1, float lastTime2) {
	float ans = MAX_DIST;
	//s1->s2->t1->t2
	if (distS1[s2] + distS2[t1] + distT1[t2] < ans && distS1[s2] / SPEED * 3600 < lastTime2) {
		ans = distS1[s2] + distS2[t1] + distT1[t2];
	}
	//s1->s2->t2->t1
	if (distS1[s2] + distS2[t2] + distT2[t1] < ans && distS1[s2] / SPEED * 3600 < lastTime2) {
		ans = distS1[s2] + distS2[t2] + distT2[t1];
	}
	//s2->s1->t1->t2
	if (distS2[s1] + distS1[t1] + distT1[t2] < ans && distS2[s1] / SPEED * 3600 < lastTime1) {
		ans = distS2[s1] + distS1[t1] + distT1[t2];
	}
	//s2->s1->t2->t1
	if (distS2[s1] + distS1[t2] + distT2[t1] < ans && distS2[s1] / SPEED * 3600 < lastTime1) {
		ans = distS2[s1] + distS1[t2] + distT2[t1];
	}

	return ans;

}

QueryInfo setQueryInfo(int startId, int endId, float ratio, int latestTime, int queryId) {
	QueryInfo newQuery;
	newQuery.endId      = endId;
	newQuery.startId    = startId;
	newQuery.maxRatio   = ratio;
	newQuery.queryId    = queryId;
	newQuery.latestTime = latestTime;

	time_t rawtime;
	time(&rawtime);
	newQuery.queryTime = rawtime;

	return newQuery;
}

void initMap() {
	for (int i = 0; i < MAX_PAIRS; i++)
		for (int j = 0; j < MAX_PAIRS; j++)
			map[i][j] = 0;
	for (int i = 0; i < MAX_PAIRS; i++)
		edge[i].clear();
}

int findb(int a){
	return belong[a] == a ? a : belong[a] = findb(belong[a]);
}

void together(int a, int b){
	a = findb(a);
	b = findb(b);
	if (a != b) belong[a] = b;
}

int findLCA(int x, int y){
	static int t = 0;
	t++;
	while (1){
		if (x != -1){
			x = findb(x);
			if (visited[x] == t)
				return x;
			visited[x] = t;
			if (spouse[x] != -1)
				x = Next[spouse[x]];
			else x = -1;
		}
		swap(x, y);
	}
}

void goup(int a, int p){
	while (a != p){
		int b = spouse[a], c = Next[b];
		if (findb(c) != p) Next[c] = b;
		if (mark[b] == 2)  mark[Q[bot++] = b] = 1;
		if (mark[c] == 2)  mark[Q[bot++] = c] = 1;
		together(a, b);
		together(b, c);
		a = c;
	}
}

void findaugment(int s, int N){
	for (int i = 0;i < N;i++){
		Next[i]    = -1;
		belong[i]  = i;
		mark[i]    = 0;
		visited[i] = -1;
	}
	Q[0]    = s;
	bot     = 1;
	mark[s] = 1;
	for (int head = 0;spouse[s] == -1 && head < bot; head++){
		int x = Q[head];
		for (int i = 0; i < edge[x].size(); i++){
			int y = edge[x][i];
			if (spouse[x] != y && findb(x) != findb(y) && mark[y] != 2){
				if (mark[y] == 1){
					int p = findLCA(x,y);
					if (findb(x) != p) Next[x]=y;
					if (findb(y) != p) Next[y]=x;
					goup(x, p);
					goup(y, p);
				} else if (spouse[y] == -1){
					Next[y] = x;
					for (int j = y; j != -1;){
						int k = Next[j];
						int l = spouse[k];
						spouse[j] = k;
						spouse[k] = j;
						j = l;
					}
					break;
				} else{
					Next[y] = x;
					mark[Q[bot++] = spouse[y]] = 1;
					mark[y] = 2;
				}
			}
		}
	}
}


void constructMapForQuery(vector<QueryInfo> queries, int n, int caseId) {
	initMap();

	vector<float> pudongDist, antiPudongDist;
	int pudongId = 31948;

	antiSPFA(pudongId);
	SPFA(pudongId);

	for (int i = 0; i < ALL_NODES; i++) {
		antiPudongDist.push_back(antiMinDist[i]);
	}

	for (int i = 0; i < ALL_NODES; i++) {
		pudongDist.push_back(minDist[i]);
	}

	for (int i = 0; i < n - 1; i ++)
		for (int j = i + 1; j < n; j ++)
			if (spouse[i] == -1 && spouse[j] == -1 && i != j) {
				QueryInfo query1, query2;
				query1 = queries[i];
				query2 = queries[j];
				float shareDist, dist1, dist2;
				if (caseId == 1) {
					shareDist = shareDistance(pudongDist, query1.distT, pudongDist, query2.distT, pudongId, query1.endId, pudongId, query2.endId, MAX_TIME, MAX_TIME);
					dist1 = pudongDist[query1.endId];
					dist2 = pudongDist[query2.endId];

				} else if (caseId == 2) {
					float remainTime1 = query1.latestTime * 60;
					float remainTime2 = query2.latestTime * 60;
					shareDist = shareDistance(query1.distS, pudongDist, query2.distS, pudongDist, query1.startId, pudongId, query2.startId, pudongId, remainTime1, remainTime2);
					dist1 = antiPudongDist[query1.startId];
					dist2 = antiPudongDist[query2.startId];
				} else if (caseId == 3) {
					float remainTime1 = query1.latestTime * 60;
					float remainTime2 = query2.latestTime * 60;
					shareDist = shareDistance(query1.distS, query1.distT, query2.distS, query2.distT, query1.startId, query1.endId, query2.startId, query2.endId, remainTime1, remainTime2);
					dist1 = query1.distS[query1.endId];
					dist2 = query2.distS[query2.endId];
				}

				float curRatio = getSaveRatio(dist1, dist2, shareDist);
				if (curRatio < query1.maxRatio && curRatio < query2.maxRatio) {
					map[i][j] = 1;
					map[j][i] = 1;
					edge[i].push_back(j);
					edge[j].push_back(i);
				}
			}
}


void testForCase1() {
	list<QueryInfo> queries;
	ifstream fin("test1.txt");
	int pudongId = 31948;

	SPFA(pudongId);
	vector<float> pudongDist;

	for (int i = 0; i < ALL_NODES; i++) {
		pudongDist.push_back(minDist[i]);
	}
	int endId;
	float ratio;
	int testPairs = 25;
	int allPairs  = 50;

	int ans[200];
	int i = 0;
	int latestTime = 0;

	for (int j = 0; j < allPairs; j++) ans[j] = -1;
	time_t rawtime;
	float allRatio = 0;

	for (int i = 0; i < allPairs; i++) {
		fin>>endId>>ratio;
		cout<<endId<<" "<<ratio<<endl;
		QueryInfo newQuery;
		newQuery = setQueryInfo(pudongId, endId, ratio, latestTime, i);
		SPFA(endId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distT.push_back(minDist[j]);
		}

		list<QueryInfo>::iterator itor = queries.begin();
		bool flag = false;
		while (itor != queries.end()) {
			time(&rawtime);
			if (true) {					
				float shareDist = shareDistance(pudongDist, (*itor).distT, pudongDist, newQuery.distT, pudongId, (*itor).endId, pudongId, newQuery.endId, MAX_TIME, MAX_TIME);
				float dist1 = pudongDist[(*itor).endId];
				float dist2 = pudongDist[endId];
				float curRatio = getSaveRatio(dist1, dist2, shareDist);
				if (curRatio < (*itor).maxRatio && curRatio < ratio) {
					ans[(*itor).queryId] = i;
					ans[i] = (*itor).queryId;
					if (i < testPairs) allRatio += curRatio;
					if ((*itor).queryId < testPairs) allRatio += curRatio;
					queries.erase(itor);
					flag = true;
					cout<<curRatio<<endl;
					break;
				} else {
					itor ++;
				}
			} else {itor ++;}
		}
		if (!flag) {
			queries.push_back(newQuery);
		}
	}
	fin.close();

	ofstream fout("ans1.txt");
	int success = 0;
	int waitNum = 0;
	for (int i = 0; i < testPairs; i++) {
		if (ans[i] != -1) {
			success ++;
			if (i < ans[i]) {
				waitNum += ans[i] - i;
				fout<<i<<"<->"<<ans[i]<<" "<<ans[i] - i<<endl;
			} else {
				fout<<i<<":"<<0<<endl;
			}

		}
	}
	fout<<"前"<<testPairs<<"个请求中成功请求数："<<success<<endl;
	success = 0;
	for (int i = testPairs; i < allPairs; i++)
		if (ans[i] != -1) {
			success ++;
		}
	fout<<"第"<<testPairs<<"到第"<<allPairs<<"个请求中成功请求数:"<<success<<endl;
	fout<<"平均Ratio: "<<allRatio / success<<endl;
	fout<<"平均等待请求数："<<waitNum / success<<endl;
	fout.close();
}

void matchTest1() {

	vector<QueryInfo> queries;
	ifstream fin("test1.txt");
	int pudongId = 31948;

	for (int i = 0; i < MAX_PAIRS; i++) spouse[i] = -1;

	SPFA(pudongId);
	vector<float> pudongDist;

	for (int i = 0; i < ALL_NODES; i++) {
		pudongDist.push_back(minDist[i]);
	}
	int endId;
	float ratio;
	int testPairs = 25;
	int allPairs  = 50;

	int ans[200];
	int i = 0;
	int latestTime = 0;

	for (int j = 0; j < allPairs; j++) spouse[j] = -1;
	time_t rawtime;
	float allRatio = 0;

	for (int i = 0; i < allPairs; i++) {
		fin>>endId>>ratio;
		cout<<endId<<" "<<ratio<<endl;
		QueryInfo newQuery;
		newQuery = setQueryInfo(pudongId, endId, ratio, latestTime, i);
		SPFA(endId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distT.push_back(minDist[j]);
		}
		queries.push_back(newQuery);
	}

	fin.close();

	cout<<"Constructing the map..."<<endl;
	int n = queries.size();
	constructMapForQuery(queries, queries.size(), 1);


	for (int i=0;i<n;i++)
		if (spouse[i]==-1) {
			cout<<"findaugment:"<<i<<endl;
			findaugment(i, n);
		}

	ofstream fout("ans11.txt");
	int success = 0;
	int waitNum = 0;
	for (int i = 0; i < testPairs; i++) {
		if (spouse[i] != -1) {
			success ++;
			fout<<i<<"<->"<<spouse[i]<<endl;
		}
	}
	fout<<"前"<<testPairs<<"个请求中成功请求数："<<success<<endl;
	success = 0;
	for (int i = testPairs; i < allPairs; i++) {
		if (spouse[i] != -1) success ++;
	} 
	fout<<"第"<<testPairs<<"到第"<<allPairs<<"个请求中成功请求数:"<<success<<endl;
	fout.close();
}

void testForCase2() {
	list<QueryInfo> queries;
	ifstream fin("test2.txt");
	//	int pudongId = 411445;
	int pudongId = 31948;	
	antiSPFA(pudongId);
	SPFA(pudongId);
	vector<float> pudongDist, antiPudongDist;
	time_t rawtime;
	int testPairs = 100;
	int allPairs  = 200;
	int ans[200];
	for (int i = 0; i < allPairs; i++) ans[i] = -1;

	for (int i = 0; i < ALL_NODES; i++) {
		antiPudongDist.push_back(antiMinDist[i]);
	}
	for (int i = 0; i < ALL_NODES; i++) {
		pudongDist.push_back(minDist[i]);
	}

	int startId, latestTime;
	float ratio, allRatio = 0;


	int i = 0;
	for (int i = 0; i < allPairs; i++) {
		fin>>startId>>latestTime>>ratio;
		cout<<startId<<" "<<latestTime<<" "<<ratio<<endl;
		//插入一个新的查询到队列中 
		QueryInfo newQuery = setQueryInfo(startId, pudongId,ratio, latestTime, i);
		SPFA(startId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distS.push_back(minDist[j]);
		}


		list<QueryInfo>::iterator itor = queries.begin();
		bool flag = false;
		while (itor != queries.end()) {
			float dist1 = antiPudongDist[(*itor).startId];
			float dist2 = antiPudongDist[startId];

			float remainTime1 = (*itor).latestTime * 60 - (rawtime - (*itor).queryTime);
			float remainTime2 = latestTime * 60;
			float shareDist = shareDistance((*itor).distS, pudongDist, newQuery.distS, pudongDist, (*itor).startId, pudongId, newQuery.startId, pudongId, remainTime1, remainTime2);

			float curRatio = getSaveRatio(dist1, dist2, shareDist);
			if (curRatio < (*itor).maxRatio && curRatio < ratio) {
				flag = true;
				ans[(*itor).queryId] = i;
				ans[i] = (*itor).queryId;
				cout<<curRatio<<endl;
				if (i < testPairs) allRatio += curRatio;
				if ((*itor).queryId < testPairs) allRatio += curRatio;
				queries.erase(itor);
				break;
			} else {
				itor ++;
			}

		}
		if (!flag) {
			queries.push_back(newQuery);
		}
	}
	fin.close();

	ofstream fout("ans2.txt");
	int success = 0;
	int waitNum = 0;
	for (int i = 0; i < testPairs; i++) {
		if (ans[i] != -1) {
			success ++;
			if (i < ans[i]) {
				waitNum += ans[i] - i;
				fout<<i<<"<->"<<ans[i]<<" "<<ans[i] - i<<endl;
			} else {
				fout<<i<<":"<<0<<endl;
			}

		}
	}
	fout<<"前"<<testPairs<<"个请求中成功请求数："<<success<<endl;
	success = 0;
	for (int i = testPairs; i < allPairs; i++)
		if (ans[i] != -1) {
			success ++;
		}
	fout<<"第"<<testPairs<<"到第"<<allPairs<<"个请求中成功请求数:"<<success<<endl;
	fout<<"平均Ratio: "<<allRatio / success<<endl;
	fout<<"平均等待请求数："<<waitNum / success<<endl;
	fout.close();

}



void matchTest2() {

	vector<QueryInfo> queries;
	ifstream fin("test2.txt");
	int pudongId = 31948;

	for (int i = 0; i < MAX_PAIRS; i++) spouse[i] = -1;

	SPFA(pudongId);
	vector<float> pudongDist, antiPudongDist;

	for (int i = 0; i < ALL_NODES; i++) {
		pudongDist.push_back(minDist[i]);
	}
	
	antiSPFA(pudongId);
	for (int i = 0; i < ALL_NODES; i++) {
		antiPudongDist.push_back(antiMinDist[i]);
	}
	int startId;
	float ratio;
	int testPairs = 100;
	int allPairs  = 200;

	int ans[200];
	int i = 0;

	for (int j = 0; j < allPairs; j++) spouse[j] = -1;
	time_t rawtime;
	float allRatio = 0;
	int latestTime;

	for (int i = 0; i < allPairs; i++) {
		fin>>startId>>latestTime>>ratio;
		cout<<startId<<" "<<latestTime<<" "<<ratio<<endl;
		QueryInfo newQuery;
		newQuery = setQueryInfo(startId, pudongId, ratio, latestTime, i);
		SPFA(startId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distS.push_back(minDist[j]);
		}
		queries.push_back(newQuery);
	}

	fin.close();

	cout<<"Constructing the map..."<<endl;
	int n = queries.size();
	constructMapForQuery(queries, queries.size(), 2);


	for (int i = 0; i < n; i++)
		if (spouse[i] == -1) {
			cout<<"findaugment:"<<i<<endl;
			findaugment(i, n);
		}

	ofstream fout("ans22.txt");
	int success = 0;
	int waitNum = 0;
	for (int i = 0; i < testPairs; i++) {
		if (spouse[i] != -1) {
			success ++;
			fout<<i<<"<->"<<spouse[i]<<endl;
		}
	}
	fout<<"前"<<testPairs<<"个请求中成功请求数："<<success<<endl;
	success = 0;
	for (int i = testPairs; i < allPairs; i++) {
		if (spouse[i] != -1) success ++;
	} 
	fout<<"第"<<testPairs<<"到第"<<allPairs<<"个请求中成功请求数:"<<success<<endl;
	fout.close();
}

void testForCase3() {
	list<QueryInfo> queries;
	ifstream fin("test3.txt");

	time_t rawtime;
	int testPairs = 100;
	int allPairs  = 200;
	int ans[250];
	for (int i = 0; i < allPairs; i++) ans[i] = -1;

	int startId, endId, latestTime;
	float ratio, allRatio = 0;


	for (int i = 0; i < allPairs; i++) {
		fin>>startId>>endId>>latestTime>>ratio;
		cout<<startId<<" "<<endId<<" "<<latestTime<<" "<<ratio<<endl;
		//插入一个新的查询到队列中 
		QueryInfo newQuery = setQueryInfo(startId, endId, ratio, latestTime, i);
		SPFA(startId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distS.push_back(minDist[j]);
		}
		SPFA(endId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distT.push_back(minDist[j]);
		}


		list<QueryInfo>::iterator itor = queries.begin();
		bool flag = false;
		while (itor != queries.end()) {
			QueryInfo selQuery = *itor;

			float dist1 = selQuery.distS[selQuery.endId];
			float dist2 = newQuery.distS[endId];
			float remainTime1 = selQuery.latestTime * 60 - (rawtime - selQuery.queryTime);
			float remainTime2 = latestTime * 60;
			float shareDist = shareDistance(selQuery.distS, selQuery.distT, newQuery.distS, newQuery.distT, selQuery.startId, selQuery.endId, newQuery.startId, newQuery.endId, remainTime1, remainTime2);

			float curRatio = getSaveRatio(dist1, dist2, shareDist);
			if (curRatio < selQuery.maxRatio && curRatio < ratio) {
				flag = true;
				ans[selQuery.queryId] = i;
				ans[i] = selQuery.queryId;
				cout<<curRatio<<endl;
				if (i < testPairs) allRatio += curRatio;
				if (selQuery.queryId < testPairs) allRatio += curRatio;
				queries.erase(itor);
				break;
			} else {
				itor ++;
			}

		}
		if (!flag) {
			queries.push_back(newQuery);
		}
	}
	fin.close();

	ofstream fout("ans3.txt");
	int success = 0;
	int waitNum = 0;
	for (int i = 0; i < testPairs; i++) {
		if (ans[i] != -1) {
			success ++;
			if (i < ans[i]) {
				waitNum += ans[i] - i;
				fout<<i<<"<->"<<ans[i]<<" "<<ans[i] - i<<endl;
			} else {
				fout<<i<<":"<<0<<endl;
			}

		}
	}
	fout<<"前"<<testPairs<<"个请求中成功请求数："<<success<<endl;
	success = 0;
	for (int i = testPairs; i < allPairs; i++)
		if (ans[i] != -1) {
			success ++;
		}
	fout<<"第"<<testPairs<<"到第"<<allPairs<<"个请求中成功请求数:"<<success<<endl;
	fout<<"平均Ratio: "<<allRatio / success<<endl;
	fout<<"平均等待请求数："<<waitNum / success<<endl;
	fout.close();
}

void matchTest3() {

	vector<QueryInfo> queries;
	ifstream fin("test3.txt");
	int pudongId = 31948;

	for (int i = 0; i < MAX_PAIRS; i++) spouse[i] = -1;

	SPFA(pudongId);
	vector<float> pudongDist, antiPudongDist;

	for (int i = 0; i < ALL_NODES; i++) {
		pudongDist.push_back(minDist[i]);
	}
	
	antiSPFA(pudongId);
	for (int i = 0; i < ALL_NODES; i++) {
		antiPudongDist.push_back(antiMinDist[i]);
	}
	int startId, endId;
	float ratio;
	int testPairs = 100;
	int allPairs  = 200;

	int ans[200];
	int i = 0;

	for (int j = 0; j < allPairs; j++) spouse[j] = -1;
	time_t rawtime;
	float allRatio = 0;
	int latestTime;

	for (int i = 0; i < allPairs; i++) {
		fin>>startId>>endId>>latestTime>>ratio;
		cout<<startId<<" "<<endId<<" "<<latestTime<<" "<<ratio<<endl;
		QueryInfo newQuery;
		newQuery = setQueryInfo(startId, endId, ratio, latestTime, i);
		SPFA(startId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distS.push_back(minDist[j]);
		}
		antiSPFA(endId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distT.push_back(antiMinDist[j]);
		}
		queries.push_back(newQuery);
	}

	fin.close();

	cout<<"Constructing the map..."<<endl;
	int n = queries.size();
	constructMapForQuery(queries, queries.size(), 3);


	for (int i = 0; i < n; i++)
		if (spouse[i] == -1) {
			cout<<"findaugment:"<<i<<endl;
			findaugment(i, n);
		}

	ofstream fout("ans33.txt");
	int success = 0;
	int waitNum = 0;
	for (int i = 0; i < testPairs; i++) {
		if (spouse[i] != -1) {
			success ++;
			fout<<i<<"<->"<<spouse[i]<<endl;
		}
	}
	fout<<"前"<<testPairs<<"个请求中成功请求数："<<success<<endl;
	success = 0;
	for (int i = testPairs; i < allPairs; i++) {
		if (spouse[i] != -1) success ++;
	} 
	fout<<"第"<<testPairs<<"到第"<<allPairs<<"个请求中成功请求数:"<<success<<endl;
	fout.close();
}



/*void matchTest1_new() {
	vector<QueryInfo> queries;
	ifstream fin("test1.txt");
	int pudongId = 31948;

	SPFA(pudongId);
	vector<float> pudongDist;

	for (int i = 0; i < ALL_NODES; i++) {
		pudongDist.push_back(minDist[i]);
	}
	int endId;
	float ratio;
	int testPairs = 50;
	int allPairs  = 100;

	int ans[200];
	int i = 0;
	int latestTime = 0;

	for (int j = 0; j < allPairs; j++) spouse[j] = -1;
	time_t rawtime;
	float allRatio = 0;

	for (int i = 0; i < allPairs; i++) {
		fin>>endId>>ratio;
		cout<<endId<<" "<<ratio<<endl;
		QueryInfo newQuery;
		newQuery = setQueryInfo(pudongId, endId, ratio, latestTime, i);
		SPFA(endId);
		for (int j = 0; j < ALL_NODES; j++) {
			newQuery.distT.push_back(minDist[j]);
		}
		queries.push_back(newQuery);
	}

	fin.close();

	cout<<"Constructing the map..."<<endl;
	int n = testPairs;
	constructMapForQuery(queries, testPairs, 1);


	for (int i = 0;i < n;i ++)
		if (spouse[i] == -1) {
			cout<<"findaugment:"<<i<<endl;
			findaugment(i, n);
		}

	constructMapForQuery(queries, allPairs, 1);

	for (int i = 0; i < allPairs; i++) {
		if (spouse[i] == -1) {
			findaugment(i, allPairs);
		}
	}

	ofstream fout("ans11.txt");
	int success = 0;
	int waitNum = 0;
	for (int i = 0; i < testPairs; i++) {
		if (spouse[i] != -1) {
			success ++;
			fout<<i<<"<->"<<spouse[i]<<endl;
		}
	}
	fout<<"成功请求数："<<success<<endl;
	fout.close();

}*/




int main() {
	init();

	time_t rawtime; 
	struct tm * timeinfo; 

	time(&rawtime); 
	timeinfo = localtime(&rawtime); 
	printf ("系统时间是: %s", asctime (timeinfo) ); 

	//generateTestsForCase1();
	//generateTestsForCase2();
	//generateTestsForCase3();

	//testForCase1();
	//testForCase2();
	//testForCase3();

	//matchTest1();
	//matchTest2();
	matchTest3();
	
	//matchTest1_new();

	time(&rawtime); 
	timeinfo = localtime(&rawtime); 
	printf ("系统时间是: %s", asctime (timeinfo) );

	//recycle();

	return 0;
}







