#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib_io.h"

/**********************
Author:LBL(刘炜，包灵)
school:电子科技大学
Language:C++11


命令规范：
int:i;
float:f;
double:d;
char:c;
string:s;
类成员变量:m_;
Class:C;
Struct:S;
多维数组:_arr;
一维数组:_ary;
私有成员:_;
全局变量:g_; 
常量:c_;
静态变量:s_;
指针：pt_;
成员变量指针：pt;
双重指针/指针数组:pp_
类静态变量：mS_;
类静态常量：mSC_;
bool:b;
优先队列：p
类方法:开头大写

eg:
蚂蚁类：CAnt;
路径：m_fPathCost，成员变量浮点型
**********************/

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <bitset>
#include <queue>
//#include <cfloat>

#define MAX_DEMAND_VEX_NUM 100 //必经节点最大数
#define MAX_VERTEX 2000
#define MAX_OUT_DEGREE 20
#define MAX_COST 100
#define MAX_NCOST 100
#define MYINF 0x3f3f3f3f  
//这里没有设为0x7F原因是怕最大加上其它后溢出，或变成-1,memset是以每个字节进行赋值的所以相当于memset(a,0x3f,sizeof(a))
//#define INF
#define ANT_COUNT 10

/*
存储结构用邻接表与逆邻接表
C++11中struct与class的唯一区别就是默认的访问权限，struct默认访问权限是public，class默认访问权限是private
*/
struct SArcNode
{
	int headVex,tailVex;//弧头尾结点
	SArcNode *nextLink;//弧头以及弧尾相同的链域
	int cost;//弧的权值
	//double Ncost;
	int arcId;//弧的编号

	//bool isvalid[MAX_DEMAND_NUM];//按对应的节点剪枝过后，标记是否有效
	double value;//弧中不为信息素，而是综合评价值,ln(入度)+ln(出度)+ln(权值)+ln(rand(0,100))，初始值为cost;

	SArcNode():headVex{-1},tailVex{-1},nextLink{nullptr},cost{MYINF},arcId{-1},value{} {}
};//弧结点

struct SVexNode
{
	//int vexId;//顶点编号,以数组位置表示
	//bool key;//关键点用true表示，否则为false;
	int outDegree;//原始出度
	int inDegree;//原始入度
	SArcNode *firstLink;//第一条出弧与入弧链域
	double pheromone;//信息素，必经节点才有
	//int outDe[MAX_DEMAND_NUM];//出度，按对应的必经节点进行剪枝过后的
	//int inDe[MAX_DEMAND_NUM];//入度，

	SVexNode():outDegree{},inDegree{},firstLink{nullptr} {}
};//顶点结点

class CDemand
{
public:
	int m_iStrat;
	int m_iEnd;
	int m_iNumUa;
	int m_iUnavoid_ary[MAX_DEMAND_VEX_NUM];

	CDemand():m_iNumUa{} {}
	void InitDemand(char *d);
};

class COrGraph
{
public:
	SVexNode m_SVexList_ary[MAX_VERTEX];//该有向图的顶点不超过2000个
	SArcNode* m_SArcList_ary[MAX_VERTEX*MAX_OUT_DEGREE];//指针数组，在创建邻接表时记录下弧的位置，序号是弧id
	int m_iVexNum,m_iArcNum;//有向图当前的顶点数与弧度数
	bool m_bIsReverse;//注明这个图是正向的还是逆向的，以供Init()时用
	COrGraph();
	void InitCreate( char ** topo, int edge_num);//创建邻接表
	//void Init();
};


//优先队列
class CHeapNode
{
public:
	int vex;//点编号
	float dist;

	CHeapNode(int i,float j):vex{i},dist{j} {}
	/*friend bool operator < (const CHeapNode &n1,const CHeapNode &n2)
	{
		return n1.dist < n2.dist;
	}*/
};
/*重写比较类，用于优先队列，其中operator()()为重载括号运算符,用于比较指针时*/
class CCompare
{
public:
	bool operator() (CHeapNode* p1,CHeapNode* p2)
	{
		return p1->dist > p2->dist;
	}
};

//相当于CAnt
class CRouteAnt
{
public:
	//std::priority_queue<SVexNode*, std::vector<SVexNode*>, CCompare > m_pHeap;
	std::bitset<MAX_VERTEX> m_bitVexFlag;//标记点是否经过，因为无环，所以对于一条路每个点只经过一次
	float m_iDist[MAX_VERTEX];//记录到起点的距离
	int m_iPathNode[MAX_VERTEX];//记录前驱
	int m_iPathArc[MAX_VERTEX];//记录经过的弧，与记录点一样，也是回回溯找弧
	//int m_iVisiArcNum;//记录路中弧数，可能用于信息数更新
	//int m_iKeyNum;
	int m_iVisiCriNum;
	int m_iVisiCriNumQue[MAX_DEMAND_VEX_NUM+1];//记录访问必经的序列,起点也是必经节点，所以要加1
	int m_iNowPosi;
	int m_iNextPosi;
	//int m_iPathCost;
	//CRouteAnt():m_iDist{MYINF},m_iPathNode{-1},m_iPathArc{-1},m_iVisiCriNum{0},m_iNowPosi{-1},m_iNextPosi{-1} {}
	CDemand *m_ptDe;//用于指向命令
	std::bitset<MAX_VERTEX> m_bitIsKey;//标记必经

	double m_dPheromone[MAX_DEMAND_VEX_NUM+1][MAX_DEMAND_VEX_NUM+1];//+1表示加上开始点，放在最后，终点不到任何点，所以不加入

	//参数设定：
	//static const float mCS_Alpha=0.1;//表征信息素重要程度的参数
	//static const float mCS_Beta=0.1;//表征启发式因子重要程度的参数
	static const float mCS_Rho;//全局信息素加强系数
	//static const float mCS_Q=0.1;//信息素增加强度系数
	static const float mCS_Zeta;//局部信息素挥发系数
	static const float mCS_Gamma;//大于等于1，初始化信息素为1/(gamma*minBECost)

	int m_iMinBECost;

private:
	int m_iKeyIndex[MAX_VERTEX];//用于标记必经节点在信息素数组中的索引
	//CRouteAnt():pt_De{nullptr} {}
public:
	bool Search();
	void InitReady();
	//两个点的距离
	bool DijMove(int srcV,int endV, bool isEnd);
	
	void InitCriNodeFlag();
	void InitPherIndex();//初始化对应信息素的节点的索引

	int GetKeyIndex(int i);
	void SetKeyIndex(int i,int val);
private:
	void DealMoveInfo();
	int ChooseNextCriNode();
	void FreeMem(std::priority_queue<CHeapNode*, std::vector<CHeapNode*>,CCompare > &h);//想法释放内存
	void PartPherUpdate(double &phe);
	
};

class CRoute
{
public:
	//记录蚂蚁跑的最优路
	int m_iBestArc[MAX_VERTEX];//记录经过的弧,前驱
	int m_iBestNode[MAX_VERTEX];//记录最优的路
	int m_iMinCost;
	//int m_iMinCostArcNum;//记录最小权值的弧数
	//输出时用
	int m_iBestArcNodeNum;//记录弧的数，输出的时候用
	int m_iArcResult[MAX_VERTEX];//依次记录经过的弧
	//int m_iNodeResult[MAX_VERTEX];//依次记录经过的点
	bool m_bIsHaveRoute;
	CRouteAnt m_CRAnt;//蚂蚁

	bool m_bIsWorkRoute;


	//静态变量的初始化在类定义的外面，在类中只是声明，没有分配空间，在类外定义时才分配
	//构造函数，初始化m_bIsWorkRoute,输出时用
	CRoute():m_bIsWorkRoute{true} {}
	//~CRecordNode();
	bool SearchBest();
	//void OutputBestRoute();
private:
	//void DealRoute();
	int CalcCost();
	void RecordBestAnt();
	void InitSituation();//初始化信息素
	void PherUpdate();
};

class CFutureRoute
{
public:
	CRoute R;
	static CDemand **pp_De;//用于存放命令
	static COrGraph *pt_G;

	/*检测重边，那么检测了重边之后，又怎么对路径作调整了，具体应用场景是什么，可不可以删去重边再跑，重边的权值设为非常大
	如果说这样还有重边像case1-1，那么说明其中有一条路径占用了必经点，此时它可能最短，但是可能有其它路来代替它，两条路都得以没重边。
	方案:重新跑其中一边，如果还有重边，则重新跑另外一边。
	删去重边哪个先跑，哪个后跑有影响吗，对于相同起终点，相同必经的这样的场景呢，是没有影响的
	
	注意：以评价函数来选择（初始时可以加上随机数），以真实权值为度量

	流程：
	1、跑一边最短的存上，更新cost，跑一边最短的存上
	2、检测是否有重边，并记录下来
	3、更新重边的cost为MYINF，然后跑其中一边，如果重边变小则记录。然后跑另一边，同理。
	4、对于3,可以交叉跑路，直到在次数之内选择最优的，或者直到没重边为止
	*/

	int m_iBestArc[MAX_DEMAND_NUM][MAX_VERTEX];//记录经过的弧,前驱
	int m_iBestNode[MAX_DEMAND_NUM][MAX_VERTEX];//记录最优的路的点

	int m_iTotalCost[MAX_DEMAND_NUM];
	int m_iRouteArcNum[MAX_DEMAND_NUM];//记录路径对应的弧的数量

	int m_iRouteResult[MAX_DEMAND_NUM][MAX_VERTEX];//记录路的经过的弧,有两条方便检测重边
	

	std::bitset<MAX_VERTEX*MAX_OUT_DEGREE> m_bitArcFlag;//用于检测重边来进行标记
	int m_iRepeatArc;//记录重复的边的条数
	//bool m_bAllBeFind;

	//int m_iIDe;//第几条命令

	//void Init(int iDe);//将命令值赋给对象R;
	void Search();
	void AdjustRouteNode(int j);//为了避免重路，调整
	//关于减枝，现在这种算法，减枝就是对出度入度的影响，和时间的影响，所以影响不大。
	//但出度、入度对评价函数value，但评价函数可以加随机数所以可以不剪枝

	//关于信息素不提出来成单独数组，是因为它要以demand必经单独访问，所以一个个访问并一个个更新，所以没必要把他单独成数组。
	CFutureRoute();
	~CFutureRoute();

	void CheckRepeatArc(int j);//以第j条路径与标记的bit进行比较，检查重边
	void RecordBestRoute(int iDe);//用来将CRout中的最好的路记录下来，用来检测路是否有重边

	void UpdateVal(float r);//计算弧的value
	void OutputRoute();//输出结果
private:
	void SetArcFlag(int i);//以第i条路径作为参考，设置标记位
	void DealResult(int i);//处理记录为顺序的
	//void CheckArcFlag(int j);//以第j条路径与标记的bit进行比较
	
};

void search_route(char *graph[MAX_EDGE_NUM], int edge_num, char *condition[MAX_DEMAND_NUM], int demand_num);

/*数据准备，邻接表，逆邻接表，必经节点，起点、终点、必经节点数*/
//void createOG(COrGraph *T, char *topo[MAX_EDGE_NUM], int edge_num, bool isReverse);//创建邻接表or逆邻接表
//void explain_demand(char **dm);//解析命令到unavoid,start_end;
//void swap_Rstart_end();//将start_end赋给Rstart_end调换起、终点
//void ini_unavoid();//初始化必经点标记为真,起终点结标记为真

/*类蚁群算法*/
/*分为两次跑，一次跑一条，将经过的弧标记，然后尽量不选它，不同于tsp每个点，这个是每条弧；
那么更新信息素也是以边，以点呢，我是选边，不是选点，但dij是选点的。但弧的两边有点，headvex,tailvex；
,所以把dij改为启发式的，点的优先级先不考虑，以后看看算法是否利用这个优化
但注意一个点只能经过一次，也就是说经过的点要标记，且必经节点要全部经过，最后不一定能打到路
那这种情况如何处理：直接结束更新信息素，只挥发，或特意减弱

起点A，随机选一个必经点B，作为第一个要到的点，选择之后从起点A走到B，用dij,启发式
*/

/*新算法蚁群，禁忌搜索，必经点
大体思路：
1、从出发点出发，随机选择一个必经点作为第一个到的必经点，然后轮盘选，开始初始化信息素。
2、直到终点，未找到就重新跑，找一轮选择最优秀的蚂蚁更新信息素，其它信息素挥发
3、终止条件，重复几次得到权值，或者到达最大跌代
4、还有个弧的综合评价值，由于必经点到必经点是dij，所以同样的必经序列也有很多值，那么可以更新弧的综合评价函数，由于对于多次找不到解的，说明在dij选路时有些点被占用了，所以可以更新评价函数。评价值开始等于权值。
5、信息素的更新，和初始化可以利用权重的导数

6、注意：边可能不连续
7、交叉计算
8、参数更改
*/



/*剪枝*/
//先不剪都，看看效果再剪，剪时注意两种必经，两种出入点

/*算法与想法*/
//主要以弧来计算，而不是以点来计算

/*测试打印*/
void printf_se_ua();
void printf_degree();
void printf_info();

/*
利用以前算法结合蚁群以及禁忌搜索与免疫算法，启发式的进行搜索，启发式即为有指导意义的搜索，而不是全域搜索
*/
#endif
