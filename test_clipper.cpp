#include "clipper.hpp"  
#include"clipper.cpp"
#include <gl/glut.h> 
#include<stdio.h>
#include <vector>
#include<iostream>
#include<string>
#define use_int32
struct point { double x, y; };

using namespace ClipperLib;
using namespace std;

//vector<int> ilist = { 1,2,4,5,6,7 };


vector<vector<point> >model = { {{0,0},{10.0,0},{10.0,10.0},{0,10.0}}, {{ 20.0,30.0 },{ 20.0,40.0 },{ 30.0,40.0 },{ 30.0,30.0 },{-10000,-10000},{ 40.0,30.0 },{ 40.0,40.0 },{ 50.0,40.0 },{ 50.0,30.0 }} }; //slice  ����ģ�͵ģ��ֲ�����������꣬һά��������ά�˲�������ߵ�����

vector<Paths>model2;//��С����һȦ��������ȡ����
//vector<Paths>model3;//����Ƭ��model��ת�����͵õ��ģ���ƫ��֮����������

vector<point>shellpath;//������ϵĵ�
vector<vector<point> >shellpaths;//��ĳһ������ϵ�·��

vector<vector<vector<point> > >shell1;//������������ϵ�·��
vector<vector<vector<point> > >shell;//����������ϵ�·��
//����ע�⣬model��2ά��i���j���㣩����model2��3ά��i���m��path�ϵĵ�n���㣩
void main()
{
	point p;
	Path  way;
	Paths layer;//ÿ��·������
	IntPoint p1;
	ClipperOffset co;
	Paths solution;
	for (int i = 0; i < model.size(); i++)//����model����i��
	{
		//int m = 0, n = 0;

		for (int j = 0; j < model[i].size(); j++)//����model����i���еĵ�j����
		{
			if (model[i][j].x != -10000)
			{
				p1.X = model[i][j].x * 100;
				p1.Y = model[i][j].y * 100;
				way.push_back(p1);

				//model2[i][m][n].X = (model[i][j].x * 100);
				//model2[i][m][n].Y = (model[i][j].y * 100);
				//n++;
			}
			else
			{
				layer.push_back(way);
				way.clear();
				//m++;
				//n = 0;
			}
		
		}
		layer.push_back(way);		
		way.clear();

		model2.push_back(layer);
		layer.clear();
		//cout << B[0][0].Y << endl;
		//model2.push_back(B);
		//B.clear();



//*********************������Ȧ�ĵ㸳��shell1***********************************
		for (int j = 0; j < model2[i].size(); j++)
		{
			for (int k = 0; k < model2[i][j].size(); k++)     
			{
				p.x = double(model2[i][j][k].X) / 100;
				p.y = double(model2[i][j][k].Y) / 100;

				shellpath.push_back(p);
			}
			shellpaths.push_back(shellpath);
			shellpath.clear();
		}
		shell1.push_back(shellpaths);
		shellpaths.clear();



//******************************��ƫ��***********************************************

		for (int j = 0; j < model2[i].size(); j++)//��ʱ��������model2��ĳ����ĵ�j��path��ƫ��
		{
			double area = Area(model2[i][j]) / 10000;
			co.Clear();
			co.AddPath(model2[i][j], jtRound, etClosedPolygon);   //����׼��ƫ�Ƶ�·��
			int times = 3;										  //����ƫ�ƵĴ���������Ǵ�ӡ�Ĳ���
			cout << "--layer ��" << i << "--path :" << j << endl;		//��ʾ�ڼ��㣬�ڼ���·��
			
			if (area > 10 && area < 110)
			{
				for (int a = 0; a < 5; a++)					 //����ƫ��
				{

					co.Execute(solution, -10);							//ÿ��ƫ�Ƶĺ��

					for (int n = 0; n < solution[0].size(); n++)
					{
						p.x = double(solution[0][n].X) / 100;
						p.y = double(solution[0][n].Y) / 100;


						shellpath.push_back(p);
						cout << "(" << solution[0][n].X << "," << solution[0][n].Y << ")";
					}

					p.x = double(solution[0][0].X) / 100;
					p.y = double(solution[0][0].Y) / 100;

					shellpath.push_back(p);

					co.Clear();
					co.AddPath(solution[0], jtRound, etClosedPolygon);  //׼����һ��ƫ��
					solution.clear();
				}
				cout << endl;

				shellpaths.push_back(shellpath);
				shellpath.clear();
			}
			else
			{
				for (int a = 0; a < times; a++)					 //����ƫ��
				{

					co.Execute(solution, 7.0);							//ÿ��ƫ�Ƶĺ��

					for (int n = 0; n < solution[0].size(); n++)
					{
						p.x = double(solution[0][n].X) / 100;
						p.y = double(solution[0][n].Y) / 100;


						shellpath.push_back(p);
						cout << "(" << solution[0][n].X << "," << solution[0][n].Y << ")";
					}

					p.x = double(solution[0][0].X) / 100;
					p.y = double(solution[0][0].Y) / 100;

					shellpath.push_back(p);

					co.Clear();
					co.AddPath(solution[0], jtRound, etClosedPolygon);  //׼����һ��ƫ��
					solution.clear();
				}
				cout << endl;

				shellpaths.push_back(shellpath);
				shellpath.clear();
				//	cout << solution[0][n].Y << endl;
					/*for (int k = 0; k < solution.size(); k++)//����solution�����ӵ���j��path�ĺ���
					{
						model2[i][j].insert(model2[i][j].end(), solution[k].begin(), solution[k].end());
					}*/
			}
		}
		//solution.clear();
		shell.push_back(shellpaths);
		shellpaths.clear();
	}

	for (int i = 0; i < shell.size(); i++)
	{
		
		for (int j = 0; j < shell[i].size(); j++)
		{   
			cout <<"--layer:  "<<i<< "--path: " <<j<<endl;

			double area = Area(model2[i][j])/10000;
			cout << area << endl;

			for (int k = 0; k < shell1[i][j].size(); k++)
			{
				cout << "(" << shell1[i][j][k].x << "," << shell1[i][j][k].y << ")";
			}
			cout << endl;

			for (int k = 0; k < shell[i][j].size(); k++)
			{
				cout << "(" << shell[i][j][k].x << "," << shell[i][j][k].y << ")";
			}
			cout << endl;
		}
	}



	//cout << model2[][1][3].X << endl;

	/*p1.X = model[0][0].x * 100;
	p1.Y = model[0][0].y * 100;

	p2.X = model[0][1].x * 100;
	p2.Y = model[0][1].y * 100;
	//B = { {{1,2}, {3,4} ,{5,6} ,{7,8} } };
	A.push_back(p1);
	A.push_back(p2);
	B.push_back(A);
	C.push_back(p2);
	B.push_back(C);

	cout << B[0][1].Y << endl;

	vector<Paths>model3 = { {{{6000,1},{3,4},{4,5}},{{3,2},{4,5},{5,6}}}, {{{2,1},{3,4},{4,5}},{{3,2},{4,5},{5,6}}} };
	cout << model3[0][0][0].X << endl;
	cout << model[0][1].y * 100 << endl;*/
	//draw solution ...
	//DrawPolygon(solution);
}