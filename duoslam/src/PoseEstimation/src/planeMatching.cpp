// planeMatching.cpp : Defines the entry point for the console application.
//

#include <planeMatching.h>
#include <cmath>


#define PI	3.14159265
#define MAX_PLANE_NUM		30
#define TOLERANCE_SIZE		5
#define THRD_AREA_RATIO		0.5
#define THRD_DISTANCE		1
#define THRD_ANGL			45

using namespace std;

struct PLANE{
	int ID;
	double area;
	double ratioXY;
	double normal_x;
	double normal_y;
	double normal_z;
	double center_x;
	double center_y;
	double center_z;
	double color_R;
	double color_G;
	double color_B;
	int Neighbors[100];
	int Neighbors_num;
	int common_observations[100];
};
struct MATCHID
{
	int ID_1;
	int ID_2;
};

PlaneMatching::PlaneMatching(PbMap &PBM_source, PbMap &PBM_target):
PBMSource(PBM_source),
PBMTarget(PBM_target)
{
	findMatchedPlanes(PBMSource.vPlanes, PBMTarget.vPlanes, matched_planes);
//	cout<<"Matched plane pairs: "<< matched_planes.size()<<endl;
}

void  PlaneMatching::findMaxIndex(int* vote, int num, int* max_index, int* max_num, int& max_size)
{
//	max_index = 0;
//	int max_num = 0;
	if (max_size>num)
	{
		max_size = num;
	}
	for (int i=0; i<num; i++)
	{
		for (int j=0; j<max_size; j++)
		{
			if (vote[i]>max_num[j])
			{
				max_num[j] = vote[i];
				max_index[j] = i;
				break;
			}
		}
		
		
	}
}

void ConvertPlanes(vector<Plane> PbMap_planes, vector<PLANE> &loc_planes)
{
	for(int i = 0; i< PbMap_planes.size();i++)
	{
		PLANE nowplane;
		nowplane.ID = PbMap_planes[i].id;
		nowplane.area = PbMap_planes[i].areaHull;
		nowplane.ratioXY = PbMap_planes[i].elongation;
		nowplane.normal_x = PbMap_planes[i].v3normal(0);
		nowplane.normal_y = PbMap_planes[i].v3normal(1);
		nowplane.normal_z = PbMap_planes[i].v3normal(2);
		nowplane.center_x = PbMap_planes[i].v3center(0);
		nowplane.center_y = PbMap_planes[i].v3center(1);
		nowplane.center_z = PbMap_planes[i].v3center(2);
		nowplane.color_R = PbMap_planes[i].v3colorNrgb(0);
		nowplane.color_G = PbMap_planes[i].v3colorNrgb(1);
		nowplane.color_B = PbMap_planes[i].v3colorNrgb(2);
		nowplane.Neighbors_num = PbMap_planes[i].neighborPlanes.size();

		loc_planes.push_back(nowplane);
	}
}

void SelectMatchingRes(vector<MATCHID> matches, std::map<unsigned, unsigned> &match_res)
{
	int match_cnt = 0;
	int n_real_match_cnt = 0;
	int size_RES = matches.size();
	for (int j=0; j<size_RES; j++)
	{
		while(match_cnt<size_RES && (matches[match_cnt].ID_2 == 0))
		{
			match_cnt++;
		}
		if(match_cnt>=size_RES)
			break;
		//nowPlane.neighborPlanes.insert(pair<unsigned, unsigned>(neiborid[k],observationnum[k]));
		match_res.insert(pair<unsigned, unsigned>(matches[match_cnt].ID_1, matches[match_cnt].ID_2-1));
		cout<<matches[match_cnt].ID_1<<'\t'<<'\t'<<'\t'<<matches[match_cnt].ID_2-1<<'\n';
		match_cnt++;
		n_real_match_cnt++;
	}
	cout << "Finding Match pairs number: "<<match_res.size()<<endl;
}

void  PlaneMatching::findMatchedPlanes(vector<Plane>& planes_src, vector<Plane>& planes_target, std::map<unsigned, unsigned> &match_res)
{

	vector<PLANE> planes_1;
	vector<PLANE> planes_2;
	vector<MATCHID> matches;
	ConvertPlanes(planes_src, planes_1);
	ConvertPlanes(planes_target, planes_2);

//	cout<<"I enter find match part."<<endl;

	double cos_alfa = 0;
	int size_1 = planes_1.size();
	int size_2 = planes_2.size();
	double matrix_diff_angl[MAX_PLANE_NUM][MAX_PLANE_NUM] = {0};
	double matrix_center_dis[MAX_PLANE_NUM][MAX_PLANE_NUM] = {0};
	double matrix_self_angle_1[MAX_PLANE_NUM][MAX_PLANE_NUM] = {0};
	double matrix_self_angle_2[MAX_PLANE_NUM][MAX_PLANE_NUM] = {0};
	vector<PLANE>::const_iterator iter_1 = planes_1.begin();
	
	for(int i=0;iter_1!=planes_1.end(); i++, iter_1++)
	{		
		vector<PLANE>::const_iterator iter_2 = planes_2.begin();
		for(int j=0;iter_2!=planes_2.end(); j++, iter_2++)
		{
			matrix_diff_angl[i][j] = acos((iter_1->normal_x*iter_2->normal_x + iter_1->normal_y*iter_2->normal_y + iter_1->normal_z*iter_2->normal_z) /
				sqrt(iter_1->normal_x*iter_1->normal_x + iter_1->normal_y*iter_1->normal_y + iter_1->normal_z*iter_1->normal_z) / 
				sqrt(iter_2->normal_x*iter_2->normal_x + iter_2->normal_y*iter_2->normal_y + iter_2->normal_z*iter_2->normal_z));
			matrix_diff_angl[i][j] = matrix_diff_angl[i][j] / PI * 180;

			matrix_center_dis[i][j] = sqrt((iter_1->center_x-iter_2->center_x)*(iter_1->center_x-iter_2->center_x) + 
				(iter_1->center_y-iter_2->center_y)*(iter_1->center_y-iter_2->center_y) + 
				(iter_1->center_z-iter_2->center_z)*(iter_1->center_z-iter_2->center_z));
		}	
	}
	iter_1 = planes_1.begin();

	for(int i=0;iter_1!=planes_1.end(); i++, iter_1++)
	{
		PLANE p_1 = planes_1[i];
		for(int j=i+1;j<size_1; j++)
		{
			PLANE p_1_1 = planes_1[j];
			matrix_self_angle_1[i][j] = acos((p_1.normal_x*p_1_1.normal_x + p_1.normal_y*p_1_1.normal_y + p_1.normal_z*p_1_1.normal_z) /
				sqrt(p_1.normal_x*p_1.normal_x + p_1.normal_y*p_1.normal_y + p_1.normal_z*p_1.normal_z) / 
				sqrt(p_1_1.normal_x*p_1_1.normal_x + p_1_1.normal_y*p_1_1.normal_y + p_1_1.normal_z*p_1_1.normal_z));
			matrix_self_angle_1[i][j] = matrix_self_angle_1[i][j] / PI * 180;
			matrix_self_angle_1[j][i] = matrix_self_angle_1[i][j];
		}		
	}

	vector<PLANE>::const_iterator iter_2 = planes_2.begin();

	for(int i=0;iter_2!=planes_2.end(); i++, iter_2++)
	{
		PLANE p_2 = planes_2[i];
		for(int j=i+1;j<size_2; j++)
		{
			PLANE p_2_1 = planes_2[j];
			matrix_self_angle_2[i][j] = acos((p_2.normal_x*p_2_1.normal_x + p_2.normal_y*p_2_1.normal_y + p_2.normal_z*p_2_1.normal_z) /
				sqrt(p_2.normal_x*p_2.normal_x + p_2.normal_y*p_2.normal_y + p_2.normal_z*p_2.normal_z) / 
				sqrt(p_2_1.normal_x*p_2_1.normal_x + p_2_1.normal_y*p_2_1.normal_y + p_2_1.normal_z*p_2_1.normal_z));
			matrix_self_angle_2[i][j] = matrix_self_angle_2[i][j] / PI * 180;
			matrix_self_angle_2[j][i] = matrix_self_angle_2[i][j];
		}		
	}

	
	int correspond[MAX_PLANE_NUM] = {0};
	int marked[MAX_PLANE_NUM] = {0};
	for (int x1=0; x1<size_1; x1++)
	{
		int vot_2[MAX_PLANE_NUM] = {0};
		int flag[MAX_PLANE_NUM][MAX_PLANE_NUM] = {0};
		for (int x2=0; x2<size_1; x2++)
		{
			for (int y1=0; y1<size_2; y1++)
			{
				for(int y2=y1+1; y2<size_2; y2++)
				{
					if (flag[y1][y2])
					{
						continue;
					}
					if (abs(matrix_self_angle_1[x1][x2]-matrix_self_angle_2[y1][y2])<=2)  //angle difference within 1 degree
					{
						flag[y1][y2] = 1;
						vot_2[y1]++;
						vot_2[y2]++;
					}
				}
			}
		}
		int max_num[TOLERANCE_SIZE] = {0};
		int max_index[TOLERANCE_SIZE] = {0};
		int tolerance_size = TOLERANCE_SIZE;
		findMaxIndex(vot_2, size_1, max_index, max_num, tolerance_size);
		for (int k=0; k<tolerance_size; k++)
		{
			if (marked[max_index[k]]>0 || max_num[k]<size_2/2)
			{
				continue;
			}
			double dis = matrix_center_dis[x1][max_index[k]];
			double diff_area = abs(planes_1[x1].area - planes_2[max_index[k]].area);
			double max_area = planes_1[x1].area>planes_2[max_index[k]].area ? planes_1[x1].area : planes_2[max_index[k]].area;
			double diff_ratio = diff_area / max_area;
			double diff_angl = matrix_diff_angl[x1][max_index[k]];
			if (dis<THRD_DISTANCE && diff_ratio<THRD_AREA_RATIO && diff_angl<THRD_ANGL)
			{
				correspond[x1] = max_index[k]+1;	//correspond[i]=0 means no matched node;
				marked[max_index[k]] = 1;
				break;
			}
		}
		MATCHID match_temp;
		match_temp.ID_1 = x1;
		match_temp.ID_2 = correspond[x1];
		matches.push_back(match_temp);
	}
	SelectMatchingRes(matches, match_res);
}
