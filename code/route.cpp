#include "route.h"
#include "lib_record.h"
#include <stdio.h>
//#include <iostream>

/**********************
Author:LBL(刘炜，包灵)
school:电子科技大学
Language:C++11
**********************/

/*类静态变量*/
COrGraph *CFutureRoute::pt_G=new COrGraph{};//邻接表
//COrGraph *pt_RG=new COrGraph{};//逆邻接表
//用于剪枝参考，还没有加上剪枝
//COrGraph *G_C=new COrGraph();//参考表，后期可以通过标记来参考而不用新建来参考
//COrGraph *RG_C=new COrGraph();//参考表
CDemand **CFutureRoute::pp_De=new CDemand *[MAX_DEMAND_NUM];

//类静态变量的初始化
const float CRouteAnt::mCS_Zeta=0.7;
const float CRouteAnt::mCS_Rho=0.7;
const float CRouteAnt::mCS_Gamma=1.0;
/*int CRoute::m_iGoodArc[MAX_VERTEX]{};//记录经过的弧,前驱
int CRoute::m_iGoodPath[MAX_VERTEX]{};//记录最优的路

int CRoute::m_iGoodArcNodeNum{};//记录弧的数，输出的时候用
int CRoute::m_iPathResult[MAX_VERTEX]{};//依次记录经过的弧
int CRoute::m_iNodeResult[MAX_VERTEX]{};//依次记录经过的点
*/
//int unavoid[MAX_DEMAND_NUM][MAX_DEMAND_VEX_NUM];//必经要求与必经节点的对应数组
//int start_end[MAX_DEMAND_NUM][3];//起点，终点，必经点数
//int Rstart_end[MAX_DEMAND_NUM][3];

//bitset<MAX_VERTEX> mark1;//标记是否剪枝，如没有标记的话，对必经剪枝是否可能剪的没有路了，先不标记
//bitset<MAX_VERTEX> mark2;

//返回指定范围内的随机浮点数  
int rnd(int nLow,int nUpper)  
{  
    return nLow+(nUpper-nLow)*(rand()/(RAND_MAX+1.0));  
}
double rnd(double dbLow,double dbUpper)
{  
    double dbTemp=rand()/((double)RAND_MAX+1.0);  
    return dbLow+dbTemp*(dbUpper-dbLow);  
} 
//你要完成的功能总入口
void search_route(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num)
{
	srand(time(0));
	//解析命令
	for(int i=0;i<MAX_DEMAND_NUM;++i)
	{
		CFutureRoute::pp_De[i]=new CDemand;
		CFutureRoute::pp_De[i]->InitDemand(demand[i]);
	}
	//
	//创建邻接表并初始化
	//CFutureRoute::pt_G->m_bIsReverse=false;//创建邻接表
	CFutureRoute::pt_G->InitCreate(topo, edge_num);
	//CRoute F;
	CFutureRoute F;

	F.Search();

	printf_se_ua();
	//printf_degree();
	//printf_info();

    /*unsigned short result1[] = {0, 1, 2};//P'路径
    unsigned short result2[] = {5, 6, 2};//P''路径

    for (int i = 0; i < 3; i++)
    {
        record_result(WORK_PATH, result1[i]);
        record_result(BACK_PATH, result2[i]);
    }*/
}
void CFutureRoute::DealResult(int i)
{

	int vex{ pp_De[i]->m_iEnd };
	//int n=0;
	m_iRouteArcNum[i]=0;
	//为实际的弧度数
	while(m_iBestNode[i][vex] !=-1)
	{
		//光计算权值而不对结果进行整理，另外有一个最好路中的弧的个数。可以用于更新信息素时用到
		//totalCost = totalCost + CFutureRoute::pt_G->m_SArcList_ary[ m_CRAnt.m_iPathArc[vex] ]->cost ;
		m_iRouteResult[i][ m_iRouteArcNum[i] ] = m_iBestArc[ i ][vex];
		++m_iRouteArcNum[i];
		vex = m_iBestNode[i][vex];
	}
}
void CFutureRoute::OutputRoute()
{
	//WORK_PATH
	//DealResult();
	printf("路径1权值：%d\n", m_iTotalCost[0]);
	//连接出发点的弧为-1，Num中多记了个到出发点的弧
	for(int i=m_iRouteArcNum[0]-1; i>=0; --i)
	{
		record_result(WORK_PATH, m_iRouteResult[0][i]);
	}
	/*for(int i=0; i<2000; ++i)
	{
		record_result(WORK_PATH, i);
	}*/
	//m_bIsWorkRoute=false;

	//BACK_PATH
	printf("路径2权值：%d\n", m_iTotalCost[1]);
	for(int i=m_iRouteArcNum[1]-1; i>=0; --i)
	{
		//printf("第2路\n");
		record_result(BACK_PATH, m_iRouteResult[1][i]);
	}

	//输出两条路总权值与重边数
	printf("\n两条路总权值：%d\n",m_iTotalCost[0]+m_iTotalCost[1] );
	//CheckRepeatArc(0);
}
void CFutureRoute::UpdateVal(float r)
{
	
	if(r<=1.0)
	{
		for(int i=0; i<pt_G->m_iArcNum; ++i)
		{
			if(pt_G->m_SArcList_ary[i])
			{
				pt_G->m_SArcList_ary[i]->value=log( (1+pt_G->m_SVexList_ary[ pt_G->m_SArcList_ary[i]->headVex ].outDegree )
												 * (1+pt_G->m_SVexList_ary[ pt_G->m_SArcList_ary[i]->headVex ].inDegree)
												 * ( 1+ pt_G->m_SArcList_ary[i]->cost) );
				//printf("value=%.4f\n", pt_G->m_SArcList_ary[i]->value);
			}
			
		}
	}
	else
	{	
		//float ra{0.0};
		//ra=log(rnd(0,100));
		for(int i=0; i<pt_G->m_iArcNum; ++i)
		{
			if(pt_G->m_SArcList_ary[i])
			{
				pt_G->m_SArcList_ary[i]->value=log( (1+pt_G->m_SVexList_ary[ pt_G->m_SArcList_ary[i]->headVex ].outDegree )
												 * (1+pt_G->m_SVexList_ary[ pt_G->m_SArcList_ary[i]->headVex ].inDegree)
												 * ( 1+ pt_G->m_SArcList_ary[i]->cost)*rnd(0.0,r) );
				//printf("value=%.4f\n", pt_G->m_SArcList_ary[i]->value);
			}
			
		}
	}
	
}
void CFutureRoute::RecordBestRoute(int iDe)
{
	memcpy(m_iBestArc[iDe],R.m_iBestArc,MAX_VERTEX*sizeof(int));//记录弧记录
	memcpy(m_iBestNode[iDe],R.m_iBestNode,MAX_VERTEX*sizeof(int));//记录弧记录
	//memcpy(m_iRouteResult[iDe],R.m_iArcResult,MAX_VERTEX*sizeof(int));//记录点记录
	//printf("cR:m_iRouteArcNum[%d]=%d\n", iDe,R.m_iBestArcNodeNum);
	//m_iRouteArcNum[iDe]=R.m_iBestArcNodeNum;//记录对应弧数,这里是实际的弧数
	m_iTotalCost[iDe]=R.m_iMinCost;
}

void CFutureRoute::SetArcFlag(int i)
{
	//依据第i条路的弧，把m_biArcFlag标记
	//m_bitArcFlag.reset();
	printf("m_iRouteArcNum[%d]=%d\n",i,m_iRouteArcNum[i] );
	for(int k=0; k< m_iRouteArcNum[i]; ++k)
	{
		//printf("m_iRouteResult[%d][%d]=%d\n",i,k,m_iRouteResult[i][k] );
		//printf("set:m_iRouteResult[%d][%d]=%d\n",i,k,m_iRouteResult[i][k]);
		m_bitArcFlag.set(m_iRouteResult[i][k]);
	}
}
void CFutureRoute::CheckRepeatArc(int j)
{
	int i{ (j==0)?1:0 };
	//printf("i=%d\n",i );
	SetArcFlag(i);
	m_iRepeatArc=0;
	printf("m_iRouteArcNum[%d]=%d\n",j,m_iRouteArcNum[j] );
	for(int k=0; k<m_iRouteArcNum[j]; ++k)
	{
		//printf("m_iRouteResult[%d][%d]=%d\n",j,k,m_iRouteResult[j][k] );
		if(m_bitArcFlag[ m_iRouteResult[j][k] ])
		{
			//printf("m_iRouteResult[%d][%d]=%d\n",j,k,m_iRouteResult[j][k]);
			++m_iRepeatArc;
		}
	}
	printf("\n重边数为：%d\n\n",m_iRepeatArc);
}

void CFutureRoute::Search()
{
	for(int i=0;i<MAX_DEMAND_NUM;++i)
	{
		R.m_CRAnt.m_ptDe=pp_De[i];//将命令赋值给对象
		if(R.SearchBest() )
		{
			RecordBestRoute(i);
			DealResult(i);
			if(i==0)
			{
				printf("调整\n");
				//在调整路上的弧，前得先整理结果；
				AdjustRouteNode(i);
			}
		}	
	}
	/*UpdateVal(0.0);
	for(int i=0;i<MAX_DEMAND_NUM;++i)
	{
		R.m_CRAnt.m_ptDe=pp_De[i];//将命令赋值给对象
		if(R.SearchBest() )
		{
			RecordBestRoute(i);
			DealResult(i);
			if(i==0)
			{
				printf("调整\n");
				//在调整路上的弧，前得先整理结果；
				AdjustRouteNode(i);
			}
		}	
	}
	UpdateVal(100.0);
	for(int i=0;i<MAX_DEMAND_NUM;++i)
	{
		R.m_CRAnt.m_ptDe=pp_De[i];//将命令赋值给对象
		if(R.SearchBest() )
		{
			RecordBestRoute(i);
			DealResult(i);
			if(i==0)
			{
				printf("调整\n");
				//在调整路上的弧，前得先整理结果；
				AdjustRouteNode(i);
			}
		}	
	}*/
	//printf("\n两条路总权值：%d\n",m_iTotalCost[0]+m_iTotalCost[1] );
	//可以加1个判断条件
	OutputRoute();
	CheckRepeatArc(0);

}
void CFutureRoute::AdjustRouteNode(int j)
{
	for(int i=R.m_iBestArcNodeNum-1; i>=0; --i)
	{

		pt_G->m_SArcList_ary[ m_iRouteResult[j][i] ]->value=MAX_NCOST;
		//printf("%.5f\n",pt_G->m_SArcList_ary[ m_iRouteResult[j][i] ]->value );
		//record_result(WORK_PATH, m_iArcResult[i]);
	}
}
CFutureRoute::CFutureRoute()
{
	memset(m_iBestNode,-1,sizeof(m_iBestNode));//避免没解时出错
}
CFutureRoute::~CFutureRoute()
{
	for(int i=0;i<MAX_DEMAND_NUM;++i)
	{
		delete pp_De[i];
		pp_De[i]=nullptr;
	}
	delete[] pp_De;
	pp_De=nullptr;
	for(int i=0;i<MAX_VERTEX*MAX_OUT_DEGREE;++i)
	{
		if(pt_G->m_SArcList_ary[i])
		{
			delete pt_G->m_SArcList_ary[i];
			pt_G->m_SArcList_ary[i]=nullptr;
		}
		
	}
	delete pt_G;
	pt_G=nullptr;
}
//参数粗调，即调整数值范围较大的信息启发式因子α、期望启发式因子β、信息素强度Q等参数，已得到较理想的解。
/*默认α=1， Alpha表征信息素重要程度的参数
β=1，Beta表征启发式因子重要程度的参数
ρ=0.7，Rho 信息素蒸发系数
Q=100,Q 信息素增加强度系数
m为蚂蚁个数

1、全局离线更新
T(ij)(t)=(1-Rho)*T(ij)+Rho*(dt(ij));
蚁环：dt(ij)=Q/|W|,w为t循环中m只蚂蚁走的最佳路
2、状态转移
3、信息素局部更新-负反馈机制：每当一只蚂蚁由一个节点移动到另一个节点时该路径上的信息素都按下面消除一部分
Tij=(1-x)*Tij+x*t0
*/
//信息素全局更新：增强（最优的路径进行更新）
void CRoute::PherUpdate()
{
	double tmp;
	//这里为什么会减1呢for(int i=0;i<m_CRAnt.m_iVisiCriNum-1;++i)
	//因为是前者到后者的
	for(int i=0;i<m_CRAnt.m_iVisiCriNum-1;++i)
	{
		//printf("m_CRAnt.m_iKeyIndex[%d]=%d ,m_CRAnt.m_iKeyIndex[%d]=%d\n",m_CRAnt.m_iVisiCriNumQue[i+1],m_CRAnt.GetKeyIndex( m_CRAnt.m_iVisiCriNumQue[i+1] ),m_CRAnt.m_iVisiCriNumQue[i],m_CRAnt.GetKeyIndex( m_CRAnt.m_iVisiCriNumQue[i] ) );
		tmp=m_CRAnt.m_dPheromone[ m_CRAnt.GetKeyIndex( m_CRAnt.m_iVisiCriNumQue[i+1] ) ][ m_CRAnt.GetKeyIndex( m_CRAnt.m_iVisiCriNumQue[i] ) ];
		m_CRAnt.m_dPheromone[ m_CRAnt.GetKeyIndex( m_CRAnt.m_iVisiCriNumQue[i+1] ) ][ m_CRAnt.GetKeyIndex( m_CRAnt.m_iVisiCriNumQue[i] ) ]=(1.0-m_CRAnt.mCS_Rho)*tmp+m_CRAnt.mCS_Rho*(1.0*m_iMinCost/(m_CRAnt.m_iMinBECost*m_CRAnt.m_iMinBECost) );
		/*printf("m_CRAnt.m_iKeyIndex[%d]=%d ,m_CRAnt.m_iKeyIndex[%d]=%d\n",m_CRAnt.m_iVisiCriNumQue[i],m_CRAnt.m_iKeyIndex[ m_CRAnt.m_iVisiCriNumQue[i] ],m_CRAnt.m_iVisiCriNumQue[i+1],m_CRAnt.m_iKeyIndex[ m_CRAnt.m_iVisiCriNumQue[i+1] ]);
		tmp=m_CRAnt.m_dPheromone[ m_CRAnt.m_iKeyIndex[ m_CRAnt.m_iVisiCriNumQue[i] ] ][ m_CRAnt.m_iKeyIndex[ m_CRAnt.m_iVisiCriNumQue[i+1] ] ];
		m_CRAnt.m_dPheromone[ m_CRAnt.m_iKeyIndex[ m_CRAnt.m_iVisiCriNumQue[i] ] ][ m_CRAnt.m_iKeyIndex[ m_CRAnt.m_iVisiCriNumQue[i+1] ] ]=(1.0-m_CRAnt.mCS_Rho)*tmp+m_CRAnt.mCS_Rho*(1.0/m_iMinCost);
		*/
	}
}
//信息素局部更新：挥发(走过的进行挥发)
int CRouteAnt::GetKeyIndex(int i)
{
	return m_iKeyIndex[i];
}
void CRouteAnt::SetKeyIndex(int i,int val)
{
	m_iKeyIndex[i]=val;
}
void CRouteAnt::PartPherUpdate(double &phe)
{
	phe=(1-mCS_Zeta)*phe+mCS_Zeta*(1/(m_iMinBECost*ANT_COUNT));
}
//初始信息素为起点到终点的最短路的倒数吧,这里用引用吧，可能效率高些，安全些
void CRoute::InitSituation()
{
	//初始化必经，与其它无关
	m_CRAnt.InitCriNodeFlag();
	//初始化信息素与必经对应的索引与其它无关
	m_CRAnt.InitPherIndex();

	//初始化信息素
	m_iMinCost=MYINF;
	//m_iBestArcNodeNum=0;
	m_bIsHaveRoute=false;
	
	m_CRAnt.InitReady();//这个是为下面计算BEcost用的
	if(!m_CRAnt.DijMove(m_CRAnt.m_ptDe->m_iStrat, m_CRAnt.m_ptDe->m_iEnd,true))
	{
		printf("没找到初始路\n" );
		exit(-1);
	}
	m_CRAnt.m_iMinBECost=m_CRAnt.m_iDist[m_CRAnt.m_ptDe->m_iEnd];
	printf("全局最短=%.5f\n",m_CRAnt.m_iDist[m_CRAnt.m_ptDe->m_iEnd]);
	//std::cerr << m_CRAnt.m_iDist[d->m_iEnd] << std::endl;
	//由于信息素的访问是以demand中必经节点编号来的，所以不用清空，因为到时候一个个赋值就覆盖了。
	double info=1.0/(m_CRAnt.mCS_Gamma*m_CRAnt.m_iMinBECost);
	//std::cout << "info:" <<info << std::endl;
	//初始化信息素
	for(int i=0; i<m_CRAnt.m_ptDe->m_iNumUa+1; ++i)
	{
		for(int j=0;j<m_CRAnt.m_ptDe->m_iNumUa+1;++j)
		{
			m_CRAnt.m_dPheromone[i][j]=info;
			//std::cout << "m_CRAnt.m_dPheromone[i][j]=" <<m_CRAnt.m_dPheromone[i][j] << std::endl;
			//exit(-1);
		}
	}
	
	for(int i=0;i<m_CRAnt.m_ptDe->m_iNumUa+1;++i)
	{
		m_CRAnt.m_dPheromone[i][i]=0.0;
	}
	//printf("m_CRAnt.m_dPheromone[ 0 ][0]=%.5f\n", m_CRAnt.m_dPheromone[0][0]);
	//exit(-1);
	//初始化必经节点的信息素
}
//搜索最好的路
bool CRoute::SearchBest()
{
	InitSituation();
	for(int i=0;i<ANT_COUNT;++i)
	{
		if(m_CRAnt.Search())
		{
			//printf("SearchBest寻找成功\n");
			m_bIsHaveRoute=true;
			RecordBestAnt();//记录最好的路，并计算路中的弧数，不是目前最好则不记录
			//信息素更新
			PherUpdate();
		}
	}
	return m_bIsHaveRoute;
}
//计数权值并整理路径
int CRoute::CalcCost()
{
	int vex=m_CRAnt.m_ptDe->m_iEnd;
	int totalCost{0};
	//这里不能用m_BestArcNodeNum给最终，因为这是每次找到路计算弧，所以最终的路的弧要重新计算
	m_iBestArcNodeNum=0;//这里的计算只记录到当的弧为调整信息素用

	//int n=0;
	//printf("m_CRAnt.m_iBestNode[0]=%d\n",m_iBestNode[0]);
	while(m_CRAnt.m_iPathNode[vex] !=-1)
	{
		//printf("m_CRAnt.m_iPathArc[%d]=%d\n",vex, m_CRAnt.m_iPathArc[vex]);
		//光计算权值而不对结果进行整理，另外有一个最好路中的弧的个数。可以用于更新信息素时用到
		totalCost = totalCost + CFutureRoute::pt_G->m_SArcList_ary[ m_CRAnt.m_iPathArc[vex] ]->cost ;
		//m_iArcResult[ m_iBestArcNodeNum ]=m_CRAnt.m_iPathArc[vex];
		++m_iBestArcNodeNum;
		vex=m_CRAnt.m_iPathNode[vex];
	}
	return totalCost;
}
void CRoute::RecordBestAnt()
{
	//printf("m_iMinCost=%d,m_CRAnt.m_iDist[d->m_iEnd]=%d\n", m_iMinCost,m_CRAnt.m_iDist[d->m_iEnd]);
	int cost=CalcCost();
	//printf("cost=%d\n",cost );
	if( cost <m_iMinCost)
	{
		m_iMinCost=cost;
		//m_iMinCostArcNum=m_iBestArcNodeNum;
		memcpy(m_iBestArc, m_CRAnt.m_iPathArc, MAX_VERTEX*sizeof(int) );
		//printf("1m_CRAnt.m_iPathNode[0]=%d\n",m_CRAnt.m_iPathNode[0]);
		memcpy(m_iBestNode, m_CRAnt.m_iPathNode, MAX_VERTEX*sizeof(int) );
		//printf("2m_CRAnt.m_iBestNode[0]=%d\n",m_iBestNode[0]);
	}

}
void CRouteAnt::InitCriNodeFlag()
{
	m_bitIsKey.reset();
	for(int i=0;i<m_ptDe->m_iNumUa;++i)
	{
		m_bitIsKey.set(m_ptDe->m_iUnavoid_ary[i]);
	}
	//设置起点也为必经
	m_bitIsKey.set(m_ptDe->m_iStrat);
}
//初始化的意义，就是当开始新的一次搜索时要进行清空原来数据，并设置初值
void CRouteAnt::InitReady()
{
	m_bitVexFlag.reset();
	m_iVisiCriNum=1;//初始节点也为必经
	m_iNextPosi=-1;
	memset(m_iDist,0x7F,MAX_VERTEX*sizeof(float) );
	//printf("iD=%d\n",m_iDist[d->m_iEnd] );
	memset(m_iPathNode,-1,MAX_VERTEX*sizeof(int) );

	memset(m_iVisiCriNumQue,-1,(MAX_DEMAND_VEX_NUM+1)*sizeof(int) );//要初始化这个序列不然可能出错
	//没必要重新初始化弧记录，因为在搜索过程中，记录了点就会重新更新弧，而输出结果也是通过点来找到对应弧

	m_iNowPosi=m_ptDe->m_iStrat;//设置现在位置为起点
	//printf("m_iNowPosi=%d\n",m_iNowPosi );
	m_iDist[m_ptDe->m_iStrat]=0;
	m_bitVexFlag.set(m_iNowPosi);//高设置起点为经过
}
void CRouteAnt::InitPherIndex()
{
	memset(m_iKeyIndex,-1,MAX_VERTEX*sizeof(int));
	//用于标记必经节点在信息素数组中的索引
	for(int i=0;i < m_ptDe->m_iNumUa ; ++i)
	{
		m_iKeyIndex[ m_ptDe->m_iUnavoid_ary[i] ]=i;
	}
	m_iKeyIndex[m_ptDe->m_iStrat]=m_ptDe->m_iNumUa;
	//m_iKeyIndex[m_ptDe->m_iEnd]=m_ptDe->m_iNumUa + 1;
	//信息素为0;可以不用初始化因为一会儿要赋初值
	//不能用这个给浮点赋值memset(m_dPheromone,0,MAX_DEMAND_VEX_NUM*MAX_DEMAND_VEX_NUM*sizeof(double));
}
bool CRouteAnt::Search()
{
  	//int vex;
  	InitReady();
  	//InitCriNodeFlag();//初始化必经
  	//InitPherIndex();
	//printf("2m_dPheromone[ 0 ][0]=%.5f\n", m_dPheromone[0][0]);
  	int preChooseVex{-1};
  	int repeatChoose{0};
  	//m_iNowPosi=m_ptDe->m_iStrat;
  	
    //exit(-1);
    //如果蚂蚁经过的必经节点数量小于全部必经，就继续走
    //由于初始节点也标记为必经节点了，所以要+1
    while (m_iVisiCriNum < m_ptDe->m_iNumUa+1)  
    {  
    	//printf("search中 m_iVisiCriNum=%d\n",m_iVisiCriNum);
    	//有可能陷入死循环选择，因为选择了一个点却到不了，但是呢，却又只能选择这个点，或者大多数情况下都选择的这个点
    	m_iNextPosi = ChooseNextCriNode();
    	//printf("m_iNextPosi=%d\n",m_iNextPosi );
        if( m_iNextPosi!=-1 )
        {
        	//printf("选择的必经点为：%d\n",m_iNextPosi);
        	if(DijMove(m_iNowPosi,m_iNextPosi,false))
	        {
	        	//printf("Move成功\n");
	        	//一个点到另一个点，中间如果有必经的话是没有记录的好像，那么要重新记录才行
	        	//m_iVisiCriNumQue[m_iVisiCriNum]=m_iNextPosi;//编号从0开始，所以比个数小1
	        	//++m_iVisiCriNum;

	        	m_iNowPosi=m_iNextPosi;
	        	//exit(-1);
	        	//在DealMoveInfo中有相应的m_iVisiCriNumQue与，m_iVisiCriNum更新
	        	DealMoveInfo();
	        }
	        else{
	        	//没找到路时的退出条件
	        	//printf("没找到点%d到点%d的路\n",m_iNowPosi,m_iNextPosi);
	        	if(preChooseVex!=-1 && preChooseVex==m_iNextPosi)
	        	{
	        		++repeatChoose;
	        		//当重复选择某个点走不通超过四次那么就返回了
	        		if(repeatChoose>3)
	        		{
	        			return false;
	        		}
	        	}
	        	else
	        	{
					preChooseVex=m_iNextPosi;
					//repeatChoose=0;
	        	}
	        	//由于用dij找的，所以一次没找到，就不用找到了
	        }
        }   
    }
    //printf("从这里 %d 出发到 %d\n",m_iNextPosi,m_ptDe->m_iEnd );
    //必经节点走完了就到达终点
  	if(DijMove(m_iNowPosi,m_ptDe->m_iEnd,true))
  	{
  		return true;
  	}
    //完成搜索后计算走过的路径长度  
    //CalPathLength();  
	return false;
}
int CRouteAnt::ChooseNextCriNode()
{
	double sumInfo{0.0};
	int vNum{-1};

	int nowPosiIndex{m_iKeyIndex[m_iNowPosi]};
	//printf("nowPosiIndex=%d,m_iNowPosi=%d\n",nowPosiIndex,m_iNowPosi);
	/*if(nowPosiIndex>100 || nowPosiIndex< 0)
	{
		exit(-1);
	}*/

	//int vexIndex{-1};
	//printf("sumInfo=%.5f\n", sumInfo);
	//printf("m_dPheromone[ 0 ][0]=%.5f\n", m_dPheromone[0][0]);
	//exit(-1);
	//由于起点放在结尾,且只有第一次用得到，而且不可能选到起点，所以这里i就只到m_ptDe->m_iNumUa，若到m_ptDe->m_iNumUa+1会有错，可能会选到起点本身
	for(int i=0; i < m_ptDe->m_iNumUa; ++i)
	{
		if(!m_bitVexFlag[ m_ptDe->m_iUnavoid_ary[i] ])
		{
			sumInfo =sumInfo + m_dPheromone[ nowPosiIndex ][i];
			//printf("m_dPheromone[ %d ][%d]=%.6f,%d\n",nowPosiIndex,i, m_dPheromone[ nowPosiIndex ][i],m_iNowPosi);
		}
		//printf("m_dPheromone[ %d ][%d]=%.6f,%d\n",nowPosiIndex,i, m_dPheromone[ nowPosiIndex ][i],m_iNowPosi);
		//exit(-1);
	}
	//printf("sumInfo=%.5f\n", sumInfo);
	//if(sumInfo!=sumInfo){exit(-1);}，//判断是否为nan
	//exit(-1);
	//如果有信息素不为零的路则开始选择
	if(sumInfo > 0.0)
	{
		//轮盘选择
		double dbTemp=rnd(0.0,sumInfo);//不用通过概率化为1，因为都要除那也是一样的。
		//printf("sumInfo=%.5f,dbTemp=%.5f ", sumInfo,dbTemp);
		//p=pt_G->m_SVexList_ary[m_iNowPosi].firstLink;
		//这个选择的时候不用考虑终点，因为终点不到任何点
		for(int i=0; i<m_ptDe->m_iNumUa; ++i)
		{

			if(!m_bitVexFlag[ m_ptDe->m_iUnavoid_ary[i] ])
			{
				//printf("d->m_iUnavoid_ary[%d]=%d\n", i,d->m_iUnavoid_ary[i]);
				//dbTemp=dbTemp-CFutureRoute::pt_G->m_SVexList_ary[ m_ptDe->m_iUnavoid_ary[i] ].pheromone;
				dbTemp=dbTemp-m_dPheromone[ nowPosiIndex ][i];
				if(dbTemp<=0.0)
				{
					vNum=m_ptDe->m_iUnavoid_ary[i];
					//vexIndex=i;
					PartPherUpdate(m_dPheromone[nowPosiIndex][i]);
					//if(vNum==0)	{printf("i=%d,m_ptDe->m_iUnavoid_ary[i]=%d\n",i ,m_ptDe->m_iUnavoid_ary[i]);}
					break;
				}
				//printf("1dbTemp=%.5f\n", dbTemp);
				//exit(-1);
			}		
		}
	}
	else
	{
		//有信息素总量为零的点，因为他的边到的点都被经过了
		//std::cerr <<"\n此点信息素总量为：" << sumInfo <<"  经过必经点数：" << m_iVisiCriNum << std::endl;
		//std::cerr <<"\n此点信息素总量为：" << sumInfo << std::endl;
		printf("\n此点信息素总量为:%.6f\n",sumInfo );
		//最后来修正，方法回溯
		//exit(-1);
		return -1;
	}
	if(vNum==-1)
	{
		printf("有错\n");
		exit(-1);
	}
	return vNum;
}
//容器适配器也是封装的类，并用模板呈现出来，当使用时给模板具体的参数，呈现对应的类
void CRouteAnt::FreeMem(std::priority_queue<CHeapNode*, std::vector<CHeapNode*>,CCompare > &h)
{
	while(!h.empty())
	{
		CHeapNode *minHeapNode=h.top();
		delete minHeapNode;
		h.pop();
	}
}
void CRouteAnt::DealMoveInfo()
{
	m_bitVexFlag.reset();
	int vNum{m_iNowPosi};
	m_iVisiCriNum=0;//经过的必经节点重新统计，因为到一个必经节点有可能经过其它必经节点
	//m_bitVexFlag.set(vNum);
	//int preNum{vNum};
	//把经过的一条路径记录下来
	while(vNum!=-1)
	{
		m_bitVexFlag.set(vNum);
		if(m_bitIsKey[vNum])
		{
			m_iVisiCriNumQue[m_iVisiCriNum]=vNum;//从0开始标记目前最新到达的点到起点的一个序列，相当于逆序的
			++m_iVisiCriNum;
		}
		vNum=m_iPathNode[vNum];
		//vNum=preNum;
	}
	for(int i=0;i<MAX_VERTEX;++i)
	{
		if(!m_bitVexFlag[i])
		{
			m_iDist[i]=MYINF;
		}
	}
}
//dij找两个点之间的路
bool CRouteAnt::DijMove(int srcV,int endV, bool isEnd)
{
	//最小优先队列，数组的方式,以后换feib堆。
	std::priority_queue<CHeapNode*, std::vector<CHeapNode*>,CCompare > minHeap;
	//我先只找到必经节点的最短距离，那么我不打endv加进来不就可以了吗，还费事，等找了这些之后再找最后一个必经到最后一个结点的值
	//初始化堆包含所有的顶点
	//在栈里
	//printf("m_iPathNode[0]=%d\n",m_iPathNode[0]);
	//printf("%d,%d,%d,%d\n",srcV,d->m_iEnd,m_iDist[srcV],m_iDist[d->m_iEnd] );
	minHeap.push(new CHeapNode{srcV,m_iDist[srcV]});
	
	//isend是指是否为找到终点的路，不是的话就不把终点加进堆中
	//pt_G->m_SVexList_ary[srcV].key=false;//源必为flase不然就出去了
	int v,u;
	CHeapNode *minHeapNode;
	while (!minHeap.empty() ) {
		minHeapNode=minHeap.top();
		minHeap.pop();
		//MinHeapNode* minHeapNode = extractMin(minHeap);
		u = minHeapNode->vex;//注意用后释放
		delete(minHeapNode);
		if(m_bitVexFlag[u] && u!=srcV)//如果已经在最短路中，则不用再做无用功了
		{
			//printf("u=%d\n",u);
			continue;
		}

		if(u==endV)//如果找到了endV的最短路，则返回
		{
			FreeMem(minHeap);//释放内存
			return true;
		}
		SArcNode* pCrawl = CFutureRoute::pt_G->m_SVexList_ary[u].firstLink;
		while (pCrawl != NULL) {

			v = pCrawl->headVex;
			//如果u结点连接的节点v为经过的节点则跳过这一个,如果v节点为endv节点的话，由于对endv节点是最后一次加入进去，且设为flag=false;
			//检醒所连接的点都是没有在路径上的，如果在的话则找下一个，加入堆中的节点都是还没有在路径上的
			
			//如果是endv的话但是不是最后一个结点，就下一个。即一般不加入结束点
			if(v==m_ptDe->m_iEnd && !isEnd)
			{
				pCrawl = pCrawl->nextLink;
				continue;
			}
			//如果不是最后一个必经节点不可以更新endv的距离
			//不用检查点v是否在堆中，因为前面都判断了,如果没在堆中那么它的标记就为true;
			if(!m_bitVexFlag[v] && pCrawl->value + m_iDist[u] <= m_iDist[v])
			{
				//如果新的距离与原来的距离相等则随机的进行更新，否则不更新
				if(pCrawl->value + m_iDist[u]==m_iDist[v] && rnd(0.0,1.0)<0.5)
				{
					pCrawl = pCrawl->nextLink;
					continue;
				}

				m_iDist[v] = m_iDist[u] + pCrawl->value;
				m_iPathNode[v]=u;//记录前驱
				m_iPathArc[v]=pCrawl->arcId;//arc[v]表示u->v的弧
				//decreaseKey(minHeap, v, dist[v]);//这里的v是节点编号，前面pos[v]=j;
				minHeap.push(new CHeapNode{v,m_iDist[v]});	
			}
			
			pCrawl = pCrawl->nextLink;
		}
	}
	FreeMem(minHeap);
	return false;
}

//创建邻接表 当m_bIsReverse为真是创建逆邻接表，当m_bIsReverse为假是创建邻接表
COrGraph::COrGraph()
{
	memset(m_SArcList_ary,0,MAX_VERTEX*MAX_OUT_DEGREE*sizeof(SArcNode*));
	m_iVexNum=0;
	m_iArcNum=0;
	m_bIsReverse=false;
}
void COrGraph::InitCreate( char **topo, int edge_num)
{
	int val[4];
	memset(val,-1,sizeof(val));//初始化数组，用数组记录分割，val[0]为序号，val[1]为源点，val[2]为终点，val[3]为代价。
	int nv=0;

	SArcNode *p,*q;
	char *tmp;
	char sep[]=",";
	//bool isHaveVex[MAX_VERTEX]{};//C++11新特性，初始化列表，没有则为0,这里是对2000个点作标记，可以不用bitset
	std::bitset<MAX_VERTEX> isHaveVex;//会自动初始化为全0

	for(int i=0; i<edge_num; i++)
	{
		tmp=strdup(topo[i]);
		val[0]=atoi(strsep(&tmp,sep));
		val[1]=atoi(strsep(&tmp,sep));
		val[2]=atoi(strsep(&tmp,sep));
		val[3]=atoi(tmp);
		p=new SArcNode{};
		p->arcId=val[0];
		if(m_bIsReverse)
		{
			p->tailVex=val[2];
			p->headVex=val[1];//逆向连接构建逆邻接表，headVex弧的头节点，val[1]表示源节点，val[2]表示终点	
		}
		else
		{
			p->tailVex=val[1];
			p->headVex=val[2];//正向连接，构建邻接表	
		}
		
		p->cost=val[3];
		p->value=val[3];
		p->nextLink=NULL;

		++m_SVexList_ary[p->tailVex].outDegree;
		++m_SVexList_ary[p->headVex].inDegree;
		
		if(!isHaveVex[p->headVex])
		{
			//isHaveVex[p->headVex]=true;
			isHaveVex.set(p->headVex);
			++nv;
		}
		if(!isHaveVex[p->tailVex])
		{
			//isHaveVex[p->tailVex]=true;
			isHaveVex.set(p->tailVex);
			//G->m_SVexList_ary[val[1]].firstLink=p;//不能这样因为有时，这个点计数了，但是没有链接，顶点无链接不一定无计数，所以可能导至有些点无链接
			++nv;
		}
		if(m_SVexList_ary[p->tailVex].firstLink==NULL)
		{
			m_SVexList_ary[p->tailVex].firstLink=p;

		}
		else
		{ 
			q=m_SVexList_ary[p->tailVex].firstLink;
			while(q->nextLink)
			{
				q=q->nextLink;
			}
			q->nextLink=p;
		}
		m_SArcList_ary[p->arcId]=p;//建立边的指针数组
	}
	//printf("nv=%d\n",nv );
	m_iVexNum=nv;
	m_iArcNum=edge_num;
}
/***解析demand字符串为start_end以及必经点个点和相应的点***/
void CDemand::InitDemand(char *d)
{
	char sep1[]=",";
	char sep2[]="|";
	strsep(&d,sep1);//滤过第一个编号
	m_iStrat=atoi(strsep(&d,sep1));//start point
	m_iEnd=atoi(strsep(&d,sep1));//end point
	//printf("d=%s\n", d);NA

	if(d[0]=='N')
	{
		m_iNumUa=0;
		return;
		//printf("d[0]=%c\n",d[0] );
		//exit(-1);
	}
	int j=0;
	while(d)
	{
	   m_iUnavoid_ary[j++]=atoi(strsep(&d,sep2));
	};
	m_iNumUa=j;	
}

/*******************测试****************************/
void printf_se_ua()
{
	for(int i=0;i<MAX_DEMAND_NUM;++i)
	{
		//打印起点、终点、必经点数
		printf("第%d条命令，起点：%d，终点：%d，必经点数：%d\n",i,CFutureRoute::pp_De[i]->m_iStrat, CFutureRoute::pp_De[i]->m_iEnd, CFutureRoute::pp_De[i]->m_iNumUa );
		//打印必经节点
		printf("必经节点%d：",i);
		for (int j = 0; j < CFutureRoute::pp_De[i]->m_iNumUa; ++j)
		{
			printf("%d ", CFutureRoute::pp_De[i]->m_iUnavoid_ary[j]);
		}
		printf("\n");
	}
	

}
void printf_degree()
{
	printf("顶点数：%d\n弧数：%d\n", CFutureRoute::pt_G->m_iVexNum,CFutureRoute::pt_G->m_iArcNum);
	printf("出度：");
	for(int i=0;i<CFutureRoute::pt_G->m_iVexNum;++i)
	{
		printf("%d ", CFutureRoute::pt_G->m_SVexList_ary[i].outDegree);
	}
	printf("\n");
	printf("入度：");
	for(int i=0;i<CFutureRoute::pt_G->m_iVexNum;++i)
	{
		printf("%d ", CFutureRoute::pt_G->m_SVexList_ary[i].inDegree);
	}
	printf("\n");
}
void printf_info()
{
	printf("信息素：\n");
	SArcNode *p;
	for(int i=0; i<CFutureRoute::pt_G->m_iVexNum; ++i)
	{
		//打印每条弧的信息素
		p=CFutureRoute::pt_G->m_SVexList_ary[i].firstLink;
		while(p)
		{
			//printf("%.6f ", p->t);
			p=p->nextLink;
		}
	}
	printf("\n");
}

