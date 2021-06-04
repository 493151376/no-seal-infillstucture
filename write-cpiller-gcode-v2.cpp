#include <iostream>  
#include <OpenMesh/Core/IO/MeshIO.hh>  
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>  
#include <OpenMesh/Core/Mesh/Handles.hh>
#include <OpenMesh\Core\Utils\Property.hh> 
#include <math.h>
#include <string>
#include <vector>
#include <algorithm>
#include<stdio.h>
#include <gl/glut.h>  
#include "clipper.hpp"  
#include"clipper.cpp"
#define use_int32
//#include<GL/glut.h>
//#include "GL\glut.h"
//#include "GL\freeglut.h"


using namespace ClipperLib;
using namespace std;

struct point {

	double x, y;

}p, p1;

typedef unsigned char boolean;

#define eps 1e-8
#define zero(x) (((x)>0?(x):-(x))<eps)

vector<point>coord;  //缓存一层的轮廓线
vector<point>polypoint; //一层的轮廓线坐标
vector<point>tripoint;  //一层的变化线 线段坐标 成对
vector<point>new_tripoint;  //新的一层的变化线 线段坐标 成对
vector<point>interpoint; //交点坐标
vector<vector<point> >model; //slice  整个模型的，分层的轮廓线坐标，一维层数，二维此层的轮廓线点坐标
vector<vector<point> >modelfill(model); //完成的model填充线，一维层数，二维此层的变化线 线段坐标
vector<vector<vector<point> > >modfill; //完成的model填充线，一维层数，二维轮廓，三维该轮廓的变化线 线段坐标
										
										//clipper
vector<Paths>model2;//从切片（model）转换类型得到的，做偏移之后画外轮廓。
vector<point>shellpath;//存外壳上的点
vector<vector<point> >shellpaths;//存某一层外壳上的路径
vector<vector<vector<point> > >shell2;     //存某一层外壳上的路径集合  一维该层变化线数量，二维壳的层数路径，三维路径坐标

vector<vector<vector<point> > >shell1;//存最里层轮廓上的路径      一维层数，二维每层不同的截面路径，三维路径的点坐标
vector<vector<vector<vector<point>>>> shell;//存所有外壳上的路径    一维层数，二维每层不同的截面路径，三维不同截面路径壳的层数，四维壳路径点的坐标





const string file_1 = "cube.obj";
const string file_2 = "bunnyr.obj";
const string file_3 = "check.obj";


//与调节视角有关
double xs = 0;
double ys = 0;
double zs = 0;
double scale = 0;

//与调节视点有关
double sx = 0;
double sy = 0;
double sz = 0;

typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;//定义完整MyMesh类，用三角网格，自定的属性
MyMesh mesh;

//用于包围盒与截面
double Bmax_x, Bmax_y, Bmax_z, Bmin_x, Bmin_y, Bmin_z, px, py, pz;

int countt = 0;


// 读取文件的函数
void readfile(string file) {//文件读取到了mesh当中
	// 请求顶点法线 vertex normals
	mesh.request_vertex_normals();
	//如果不存在顶点法线，则报错 
	if (!mesh.has_vertex_normals())
	{
		cout << "错误：标准定点属性 “法线”不存在" << endl;
		return;
	}
	// 如果有顶点发现则读取文件 
	OpenMesh::IO::Options opt;
	if (!OpenMesh::IO::read_mesh(mesh, file, opt))
	{
		cout << "无法读取文件:" << file << endl;
		return;
	}
	else cout << "成功读取文件:" << file << endl;
	cout << endl; // 为了ui显示好看一些
				  //如果不存在顶点法线，则计算出
	if (!opt.check(OpenMesh::IO::Options::VertexNormal))
	{
		// 通过面法线计算顶点法线
		mesh.request_face_normals();
		// mesh计算出顶点法线
		mesh.update_normals();
		// 释放面法线
		mesh.release_face_normals();
	}
}

//通过简单的遍历点获得模型包围盒（与获取截面无关）
void BoundingBox() {
	MyMesh::Point pt;
	int st = 0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
		MyMesh::VertexHandle vh_i = *it;
		pt = mesh.point(vh_i);
		px = pt.data()[0];
		py = pt.data()[1];
		pz = pt.data()[2];
		if (st == 0) {
			Bmax_x = Bmin_x = px;
			Bmax_y = Bmin_y = py;
			Bmax_z = Bmin_z = pz;
			st++;
		}
		else {
			if (px > Bmax_x)Bmax_x = px; else if (px < Bmin_x)Bmin_x = px;
			if (py > Bmax_y)Bmax_y = py; else if (py < Bmin_y)Bmin_y = py;
			if (pz > Bmax_z)Bmax_z = pz; else if (pz < Bmin_z)Bmin_z = pz;
		}
	}

}

//截面函数
bool IntersectPlane(MyMesh::Point pt, MyMesh::Point pnorm) //pt截面上一点，pnorm截面法向
{
	const float ERR = 0.001;
	//参数包括 pt，pnorm，*pilist，pnum[]  具体函数原理 见 截面算法.docx
	int starte, ne, ne1, nf;
	MyMesh::Point vt1, vt2;
	//MyMesh::Face f1;
	MyMesh::HalfedgeHandle nhe;
	MyMesh::FaceHandle nhf;
	float d1, d2, sd1, sd2;
	bool *flag, suc;
	float dist, mind = 1.0e+8;

	sd1 = sd2 = -10000;
	int esize = mesh.n_halfedges();
	flag = new bool[esize];

	suc = false;


	for (MyMesh::HalfedgeIter it = mesh.halfedges_begin(); it != mesh.halfedges_end(); ++it) //遍历整个模型所有的边，有交点的把id记录在flag中
	{

		MyMesh::HalfedgeHandle hh = *it;
		int id = hh.idx();
		flag[id] = false;

		auto fromVertex = mesh.from_vertex_handle(hh);
		auto toVertex = mesh.to_vertex_handle(hh);
		vt1 = mesh.point(fromVertex);
		vt2 = mesh.point(toVertex);
		//printf("$ %.3f %.3f $\n", vt1.data()[0],vt2.data()[0]);
		d1 = pnorm.data()[0] * (vt1.data()[0] - pt.data()[0]) + pnorm.data()[1] * (vt1.data()[1] - pt.data()[1])
			+ pnorm.data()[2] * (vt1.data()[2] - pt.data()[2]);//d1到面的距离
		d2 = pnorm.data()[0] * (vt2.data()[0] - pt.data()[0]) + pnorm.data()[1] * (vt2.data()[1] - pt.data()[1])
			+ pnorm.data()[2] * (vt2.data()[2] - pt.data()[2]);

		if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0) || d1 > 0 && d2 == 0 || d1 == 0 && d2 > 0))//线段与面相交
		{
			flag[id] = true;

			vt1.data()[0] = vt1.data()[0] - pt.data()[0];
			vt1.data()[1] = vt1.data()[1] - pt.data()[1];
			vt1.data()[2] = vt1.data()[2] - pt.data()[2];       // point date minus point date 
			dist = vt1.data()[0] * vt1.data()[0] + vt1.data()[1] * vt1.data()[1] + vt1.data()[2] * vt1.data()[2];

			if (dist < mind)
			{
				nhe = hh;	//最短边
				mind = dist;//最短距离
				ne = id;    //最短所在边的编号               //  printf("ne:  %d  \n", ne);
				suc = true;
			}
		}
	}

	if (!suc)
	{
		delete[]flag;
		return false; //没有交点，这里return false，跳出整个函数
	}

	starte = ne;//标记循环起始的边

	suc = false;

	nhf = mesh.face_handle(nhe);//最短边所在面

	while (!suc)
	{
		//printf("%%%%");	

		auto fromVertex = mesh.from_vertex_handle(nhe);
		auto toVertex = mesh.to_vertex_handle(nhe);

		vt1 = mesh.point(fromVertex);
		vt2 = mesh.point(toVertex);

		d1 = pnorm.data()[0] * (vt1.data()[0] - pt.data()[0]) + pnorm.data()[1] * (vt1.data()[1] - pt.data()[1])
			+ pnorm.data()[2] * (vt1.data()[2] - pt.data()[2]);
		d2 = pnorm.data()[0] * (vt2.data()[0] - pt.data()[0]) + pnorm.data()[1] * (vt2.data()[1] - pt.data()[1])
			+ pnorm.data()[2] * (vt2.data()[2] - pt.data()[2]);
		//printf("$$$%lf %lf \n", d1, d2);
		if ((sd1 == d1) && (sd2 == d2))
		{
			flag[ne] = false;
		}
		sd1 = d1; sd2 = d2;
		//pilist[num].data()[0] = (float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[0] - vt1.data()[0]) + vt1.data()[0];
		//pilist[num].data()[1] = (float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[1] - vt1.data()[1]) + vt1.data()[1];
		//pilist[num].data()[2] = (float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[2] - vt1.data()[2]) + vt1.data()[2];

		p = { (float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[0] - vt1.data()[0]) + vt1.data()[0] ,(float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[1] - vt1.data()[1]) + vt1.data()[1] };
		coord.push_back(p);


		do {
			for (auto it = mesh.fh_begin(nhf); it != mesh.fh_end(nhf); ++it) //nhf最短边所在面,迭代这个面的边，只有3个
			{
				MyMesh::HalfedgeHandle halfnow = *it;

				const int ne1 = halfnow.idx();

				if (flag[ne1] == false || ne == ne1) continue;

				MyMesh::VertexHandle fromV = mesh.from_vertex_handle(halfnow);
				MyMesh::VertexHandle toV = mesh.to_vertex_handle(halfnow);

				vt1 = mesh.point(fromV);
				vt2 = mesh.point(toV);

				d1 = pnorm.data()[0] * (vt1.data()[0] - pt.data()[0]) + pnorm.data()[1] * (vt1.data()[1] - pt.data()[1])
					+ pnorm.data()[2] * (vt1.data()[2] - pt.data()[2]);
				d2 = pnorm.data()[0] * (vt2.data()[0] - pt.data()[0]) + pnorm.data()[1] * (vt2.data()[1] - pt.data()[1])
					+ pnorm.data()[2] * (vt2.data()[2] - pt.data()[2]);

				p = { (float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[0] - vt1.data()[0]) + vt1.data()[0] ,(float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[1] - vt1.data()[1]) + vt1.data()[1] };
				coord.push_back(p);

				MyMesh::HalfedgeHandle halfnext = mesh.opposite_halfedge_handle(halfnow);//获取反向半边

				nhf = mesh.face_handle(halfnext);//返回这个边所在的面

				int ne2 = halfnext.idx();

				flag[ne1] = flag[ne2] = false;//ne1,ne2是对向的两个半边，存过其中一个的交点就都变为false

				if (nhf.idx() == -1)
				{
					starte = ne;
					flag[ne] = false;
					break;
				}
				ne = ne2;//以对边的那个面再找下一个与面相交的线

				//pilist[num].data()[0] = (float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[0] - vt1.data()[0]) + vt1.data()[0];
				//pilist[num].data()[1] = (float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[1] - vt1.data()[1]) + vt1.data()[1];
				//pilist[num].data()[2] = (float)fabs(d1) / ((float)fabs(d1) + (float)fabs(d2))*(vt2.data()[2] - vt1.data()[2]) + vt1.data()[2];
				//printf("##%lf %lf %lf\n", pilist[num].data()[0], pilist[num].data()[1], pilist[num].data()[2]);

				break;
			}
		} while (ne != starte);

		suc = true;

		for (auto it = mesh.halfedges_begin(); it != mesh.halfedges_end(); ++it) //检索有没有第二个环
		{

			MyMesh::HalfedgeHandle hh = *it;

			int id = hh.idx();

			if (flag[id] == true)
			{
				ne = id;
				starte = ne;
				nhe = hh;
				nhf = mesh.face_handle(nhe);
				if (nhf.idx() == -1)
				{
					flag[ne] = false;
					continue;
				}
				//pilist[num].data()[0] = -10000;
				//pilist[num].data()[1] = -10000;
				//pilist[num].data()[2] = -10000;
				p = { -10000,-10000 };//两个环中间的间隔数据
				coord.push_back(p);

				suc = false;
				break;
			}
		}


	};

	delete[]flag;
	return true;
}

void findIntersect(float z)
{
	for (double i = Bmin_z; i < Bmax_z; i += z)
	{
		MyMesh::Normal vf(0, 0, 1);//截面法向
		MyMesh::Point pt; //截面上一点
		pt.data()[0] = 0; pt.data()[1] = 0; pt.data()[2] = i;
		IntersectPlane(pt, vf);//生成一层轮廓线，存入coord

		model.push_back(coord);
		coord.clear();
	}
	countt = model.size();
}

/*void clipper()
{
	Path  way;
	Paths layer;//每层路径集合
	IntPoint p1;
	ClipperOffset co;
	Paths solution;
	for (int i = 0; i < model.size(); i++)//遍历model，第i层
	{
		//int m = 0, n = 0;

		for (int j = 0; j < model[i].size(); j++)//遍历model，第i层中的第j个点
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




	//cout << model2[1][1][3].X << endl;

		for (int j = 0; j < model2[i].size(); j++)//此时遍历的是model2，某层里的第j条path做偏移
		{

			co.Clear();
			co.AddPath(model2[i][j], jtRound, etClosedPolygon);   //设置准备偏移的路径
			int times = 1;										  //设置偏移的次数，即外壳打印的层数
			cout << "--layer ：" << i << "--path :" << j << endl;		//显示第几层，第几个路径
			for (int a = 0; a < times; a++)					 //进行偏移
			{
				co.Execute(solution, 7.0);							//每次偏移的厚度
				for (int n = 0; n < solution[0].size(); n++)
				{
					cout << "(" << solution[0][n].X << "," << solution[0][n].Y << ")";
				}
				co.Clear();
				co.AddPath(solution[0], jtRound, etClosedPolygon);  //准备下一次偏移
				solution.clear();
			}



			cout << endl;


		}
		//solution.clear();
	}
}*/


void clipper()
{
	point p;
	Path  way;
	Paths layer;//每层路径集合
	IntPoint p1;
	ClipperOffset co;
	Paths solution;
	for (int i = 0; i < model.size(); i++)//遍历model，第i层
	{
		//int m = 0, n = 0;

		for (int j = 0; j < model[i].size(); j++)//遍历model，第i层中的第j个点
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


//*********************将最内圈的点赋给shell1***********************************
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



		//******************************做偏移***********************************************
        printf("%.2lf%%\r", i * 100.0 / model.size());
		for (int j = 0; j < model2[i].size(); j++)//此时遍历的是model2，某层里的第j条path做偏移
		{
			double area = Area(model2[i][j]) / 10000;
			co.Clear();
			co.AddPath(model2[i][j], jtRound, etClosedPolygon);   //设置准备偏移的路径
			int times = 2;										  //设置偏移的次数，即外壳打印的层数
			//cout << "--layer ：" << i << "--path :" << j << endl;		//显示第几层，第几个路径
			if (area < 110 && area>5)
			{
				for (int a = 0; a < 10; a++)					 //进行偏移
				{

					co.Execute(solution, -20);							//每次偏移的厚度

					for (int n = 0; n < solution[0].size(); n++)
					{
						p.x = double(solution[0][n].X) / 100;
						p.y = double(solution[0][n].Y) / 100;


						shellpath.push_back(p);
						//cout << "(" << solution[0][n].X << "," << solution[0][n].Y << ")";
					}

					p.x = double(solution[0][0].X) / 100;
					p.y = double(solution[0][0].Y) / 100;

					shellpath.push_back(p);

					co.Clear();
					co.AddPath(solution[0], jtRound, etClosedPolygon);  //准备下一次偏移
					solution.clear();

					shellpaths.push_back(shellpath);
					shellpath.clear();
				}

				shell2.push_back(shellpaths);
				shellpaths.clear();
			}
			else 
			{
				for (int a = 0; a < times; a++)					 //进行偏移
				{

				co.Execute(solution, 20);							//每次偏移的厚度

				for (int n = 0; n < solution[0].size(); n++)
				{
					p.x = double(solution[0][n].X) / 100;
					p.y = double(solution[0][n].Y) / 100;


					shellpath.push_back(p);
					//cout << "(" << solution[0][n].X << "," << solution[0][n].Y << ")";
				}

				p.x = double(solution[0][0].X) / 100;
				p.y = double(solution[0][0].Y) / 100;

				shellpath.push_back(p);

				co.Clear();
				co.AddPath(solution[0], jtRound, etClosedPolygon);  //准备下一次偏移
				solution.clear();

				shellpaths.push_back(shellpath);
				shellpath.clear();
				}

				shell2.push_back(shellpaths);
				shellpaths.clear();
			}
		}
		//solution.clear();
		shell.push_back(shell2);
		shell2.clear();
	}

   /*	for (int i = 0; i < shell1.size(); i++)                         //打印轮廓点坐标和偏移后外壳坐标
	{
		for (int j = 0; j < shell1[i].size(); j++)
		{
			cout << "--layer:  " << i << "--path: " << j << endl;
			double area = Area(model2[i][j]) / 10000;
			cout <<"arer:"<< area << endl;
			for (int k = 0; k < shell1[i][j].size(); k++)
			{
				cout << "(" << shell1[i][j][k].x << "," << shell1[i][j][k].y << ")";
			}
			cout << endl;
			for (int k = 0; k < shell[i][j].size(); k++)
			{
				cout << "times : " << k;
				for (int m = 0; m < shell[i][j][k].size(); m++)
				{				  
                      cout << "(" << shell[i][j][k][m].x << "," << shell[i][j][k][m].y << ")";
				}		
				cout << endl;
			}
			cout << endl;
		}
		cout << endl;
	}*/
	
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


//***********************************************************************以上切片一下填充****************************************************************
//*************判断线段与线段的关系函数*************************
double xmult(point p1, point p2, point p0) {
	return (p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y);
}

//判点是否在线段上,包括端点
int dot_online_in(point p, point l1, point l2) {
	return zero(xmult(p, l1, l2)) && (l1.x - p.x)*(l2.x - p.x) < eps && (l1.y - p.y)*(l2.y - p.y) < eps;
}

//判两点在线段同侧,点在线段上返回0
int same_side(point p1, point p2, point l1, point l2) {
	return xmult(l1, p1, l2)*xmult(l1, p2, l2) > eps;
}

//判两直线平行
int parallel(point u1, point u2, point v1, point v2) {
	return zero((u1.x - u2.x)*(v1.y - v2.y) - (v1.x - v2.x)*(u1.y - u2.y));
}

//判三点共线
int dots_inline(point p1, point p2, point p3) {
	return zero(xmult(p1, p2, p3));
}

//判两线段相交,包括端点和部分重合
int intersect_in(point u1, point u2, point v1, point v2) {
	if (!dots_inline(u1, u2, v1) || !dots_inline(u1, u2, v2))
		return !same_side(u1, u2, v1, v2) && !same_side(v1, v2, u1, u2);
	return dot_online_in(u1, v1, v2) || dot_online_in(u2, v1, v2) || dot_online_in(v1, u1, u2) || dot_online_in(v2, u1, u2);
}

//计算两线段交点,请判线段是否相交(同时还是要判断是否平行!)
point intersection(point u1, point u2, point v1, point v2) {
	point ret = u1;
	double t = ((u1.x - v1.x)*(v1.y - v2.y) - (u1.y - v1.y)*(v1.x - v2.x))
		/ ((u1.x - u2.x)*(v1.y - v2.y) - (u1.y - u2.y)*(v1.x - v2.x));
	ret.x += (u2.x - u1.x)*t;
	ret.y += (u2.y - u1.y)*t;
	return ret;
}

//主函数，有交点存交点，没交点无操作
void inter(point u1, point u2, point v1, point v2)
{
	point ans;
	if (parallel(u1, u2, v1, v2) || !intersect_in(u1, u2, v1, v2))
	{

	}
	else {
		ans = intersection(u1, u2, v1, v2);
		interpoint.push_back(ans);
		//printf("交点为:(%lf,%lf)", ans.x, ans.y);
	}

}

//*************判断线段与线段的关系函数*************************

int InOrOutPolygon(point a)  //判断点是否在多边形内
{
	double x0 = a.x;
	double y0 = a.y;
	int crossings = 0;
	int n = polypoint.size();
	for (int i = 0; i < n; i++)
	{
		// 点在两个x之间 且以点垂直y轴向上做射线
		double slope = (polypoint[(i + 1 + n) % n].y - polypoint[i].y) / (polypoint[(i + 1 + n) % n].x - polypoint[i].x);
		boolean cond1 = (polypoint[i].x <= x0) && (x0 < polypoint[(i + 1 + n) % n].x);
		boolean cond2 = (polypoint[(i + 1 + n) % n].x <= x0) && (x0 < polypoint[i].x);
		boolean above = (y0 < slope * (x0 - polypoint[i].x) + polypoint[i].y);
		if ((cond1 || cond2) && above) crossings++;
	}
	return (crossings % 2 != 0);    //返 回 值:  0:外 1:内
}

void getcoord(point A, point B) //将A、B两点之间的点都存入tripoint，包括A、B
{
	float xmin, xmax, y, ymin, ymax;
	float x1, x2, y1, y2;
	x1 = A.x;
	y1 = A.y;
	x2 = B.x;
	y2 = B.y;

	float a = (y2 - y1) / (x2 - x1);
	float b = y1 - x1 * a;

	if (A.x > B.x)
	{
		xmin = x2;
		ymin = x2;
		xmax = x1;
		ymax = y1;
	}
	else
	{
		xmin = x1;
		ymin = y1;
		xmax = x2;
		ymax = y2;

	}
	//p = { xmin , ymin };
	//coord.push_back(p);
	for (; xmin < xmax;)
	{
		y = a * xmin + b;
		p = { xmin , y };
		tripoint.push_back(p);
		xmin++;

	}
	p = { xmax , ymax };
	tripoint.push_back(p);

}

point mindistance(point a, point b, point c) //在b 和 c中返回离a 最近的点
{
	int dis1 = pow((a.x - b.x), 2) + pow((a.y - b.y), 2);
	int dis2 = pow((a.x - c.x), 2) + pow((a.y - c.y), 2);
	if (dis1 < dis2)return b;
	else return c;
}

double distance(point a, point b)
{
	return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

//void triangle(int i)  //第i层，从1开始，由i和w一起决定点A、B、C的位置  1层
//{
//
//	float big_a = 17.0;
//	float small_a = 4.0;
//
//	float s = 0.1;//步长,要考虑填充线的宽度，0<s<=width/2，
//
//	float distance = (big_a - small_a) / (2 * sqrt(3));
//
//	while (i  > 4 * distance / s)
//	{
//		i = i - int(4 * distance / s);
//	}
//	int condition;
//	if (i  < 1 * distance / s) condition = 1;
//
//	else if (i  < 2 * distance / s) condition = 2;
//
//	else if (i  < 3 * distance / s) condition = 3;
//
//	else if (i  < 4 * distance / s) condition = 4;
//
//	switch (condition)
//	{
//	case 1:
//	{
//		for (int m = 0; m < 150 / big_a*sqrt(3)/2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                       m*0.5*sqrt(3)*big_a + s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a + s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 - 2 * s * i };
//
//				tripoint.push_back(D); tripoint.push_back(A);
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(E);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(F);
//				tripoint.push_back(C); tripoint.push_back(A);
//
//			}
//		}
//		break;
//	}
//
//	case 2:
//	{
//		i = i - int(distance / s);
//
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,             m*0.5*sqrt(3)*big_a + distance - s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a + distance - s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + distance + small_a * sqrt(3) / 2 + 2 * s*i };
//
//				tripoint.push_back(D); tripoint.push_back(A);
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(E);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(F);
//				tripoint.push_back(C); tripoint.push_back(A);
//			}
//		}
//		break;
//	}
//
//	case 3://和1相比，横坐标不表，纵坐标取反
//	{
//		i = i - int(2 * distance / s);
//
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                     m*0.5*sqrt(3)*big_a - s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a - s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 + 2 * s * i };
//
//				tripoint.push_back(D); tripoint.push_back(A);
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(E);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(F);
//				tripoint.push_back(C); tripoint.push_back(A);
//
//			}
//		}
//		break;
//	}
//
//	case 4://与2相比，横坐标不变，纵坐标取反
//	{
//		i = i - int(3 * distance / s);
//
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a - distance + s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a - distance + s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - distance - small_a * sqrt(3) / 2 - 2 * s*i };
//
//				tripoint.push_back(D); tripoint.push_back(A);
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(E);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(F);
//				tripoint.push_back(C); tripoint.push_back(A);
//			}
//		}
//		break;
//	}
//
//	default: printf("有问题");
//		break;
//	}
//}

void triangle(int i)  //没线
{

	float big_a = 17.0;
	float small_a = 6.0;

	float s = 0.1;//步长,要考虑填充线的宽度，0<s<=width/2，

	float distance = (big_a - small_a) / (2 * sqrt(3));

	while (i > 4 * distance / s)
	{
		i = i - int(4 * distance / s);
	}
	int condition;
	if (i < 1 * distance / s) condition = 1;

	else if (i < 2 * distance / s) condition = 2;

	else if (i < 3 * distance / s) condition = 3;

	else if (i < 4 * distance / s) condition = 4;

	switch (condition)
	{
	case 1:
	{
		for (int m = 0; m < 150*2 / big_a * sqrt(3) ; m++)
		{
			for (int n = 0; n < 200 / big_a; n++) //做大横坐标，除以大边长，就是横着总共有几个三角形
			{
				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                       m*0.5*sqrt(3)*big_a + s * i };

				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a + s * i };

				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 };
				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 - 2 * s * i };


				tripoint.push_back(A); tripoint.push_back(B); 
				tripoint.push_back(B); tripoint.push_back(C);
				tripoint.push_back(C); tripoint.push_back(A);


			}
		}
		break;
	}

	case 2:
	{
		i = i - int(distance / s);

		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
		{
			for (int n = 0; n < 200 / big_a; n++)
			{
				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,             m*0.5*sqrt(3)*big_a + distance - s * i };

				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a + distance - s * i };

				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 };
				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + distance + small_a * sqrt(3) / 2 + 2 * s*i };


				tripoint.push_back(A); tripoint.push_back(B);
				tripoint.push_back(B); tripoint.push_back(C);
				tripoint.push_back(C); tripoint.push_back(A);
			}
		}
		break;
	}

	case 3://和1相比，横坐标不表，纵坐标取反
	{
		i = i - int(2 * distance / s);

		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
		{
			for (int n = 0; n < 200 / big_a; n++)
			{
				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                     m*0.5*sqrt(3)*big_a - s * i };

				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a - s * i };

				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 };
				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 + 2 * s * i };


				tripoint.push_back(A); tripoint.push_back(B);
				tripoint.push_back(B); tripoint.push_back(C);
				tripoint.push_back(C); tripoint.push_back(A);
			}
		}
		break;
	}

	case 4://与2相比，横坐标不变，纵坐标取反
	{
		i = i - int(3 * distance / s);

		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
		{
			for (int n = 0; n < 200 / big_a; n++)
			{
				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a - distance + s * i };

				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a - distance + s * i };

				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 };
				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - distance - small_a * sqrt(3) / 2 - 2 * s*i };


				tripoint.push_back(A); tripoint.push_back(B);
				tripoint.push_back(B); tripoint.push_back(C);
				tripoint.push_back(C); tripoint.push_back(A);
			}
		}
		break;
	}

	default: printf("有问题");
		break;
	}
}

//void triangle(int i)  //没线,试图解决拉丝问题，就是先画所有的横，再画所有的丿，再画所有的捺
//{
//
//	float big_a = 17.0;
//	float small_a = 6.0;
//
//	int m = 0, n = 0;
//	int M, N;
//	N = 10;
//	M = N;
//
//	float s = 0.1;//步长,要考虑填充线的宽度，0<s<=width/2，
//
//	float distance = (big_a - small_a) / (2 * sqrt(3));
//
//	while (i > 4 * distance / s)
//	{
//		i = i - int(4 * distance / s);
//	}
//	int condition;
//	if (i < 1 * distance / s) condition = 1;
//
//	else if (i < 2 * distance / s) condition = 2;
//
//	else if (i < 3 * distance / s) condition = 3;
//
//	else if (i < 4 * distance / s) condition = 4;
//
//	switch (condition)
//	{
//	case 1:
//	{
//
//		for (; m <= M; m++) //一
//		{
//			if (m % 2 == 0)
//			{
//				for (int n = 0; n <= N; n++)
//				{
//					point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                       m*0.5*sqrt(3)*big_a + s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a + s * i };
//					tripoint.push_back(B); tripoint.push_back(C);
//				}
//			}
//			else
//			{
//				for (int n = N; n >= 0; n--)
//				{
//					point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                       m*0.5*sqrt(3)*big_a + s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a + s * i };
//					tripoint.push_back(C); tripoint.push_back(B);
//				}
//			}
//		}
//		m = 0, n = 0;
//		for (; n <= N; n++) //丿
//		{
//			if (n % 2 == 0)
//			{
//				for (int m = 0; m <= M; m++)
//				{
//					point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 - 2 * s * i };
//					point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                       m*0.5*sqrt(3)*big_a + s * i };
//					tripoint.push_back(B); tripoint.push_back(A);
//				}
//			}
//			else
//			{
//				for (int m = M ; m >= 0; m--)
//				{
//					point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 - 2 * s * i };
//					point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                       m*0.5*sqrt(3)*big_a + s * i };
//					tripoint.push_back(A); tripoint.push_back(B);
//				}
//			}
//		}
//	
//		for (int j = 1; j <= 2*M+1; j++)//这里是斜着遍历捺
//		{
//			if (j <= M + 1)
//			{
//				if (j % 2 == 0)
//				{
//					n = N - j + 1;
//					m = M;
//					goto even1;
//				}
//				else
//				{
//					n = N;
//					m = M - j + 1;
//					goto odd1;
//				}
//			}
//			else
//			{
//				if (j % 2 == 0)
//				{
//					n = 0;
//					m = 2 * M + 1 - j;
//					goto even1;
//				}
//				else
//				{
//					n = 2 * M + 1 - j;
//					m = 0;
//					goto odd1;
//				}
//			}
//
//		even1:
//			do//奇数行要反捺，和前面的提对应，
//			{
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 - 2 * s * i };
//				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a + s * i };
//				tripoint.push_back(A); tripoint.push_back(C);
//				
//				m--;
//				n++;
//			} while (m >= 0 && n <= N);
//			continue;
//		odd1:
//			do
//			{
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 - 2 * s * i };
//				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a + s * i };
//				tripoint.push_back(C); tripoint.push_back(A);
//				
//				m++;
//				n--;
//			} while (m <= M && n >= 0);
//
//		}
//
//		break;
//	}
//
//	case 2:
//	{
//		i = i - int(distance / s);
//
//		/*for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,             m*0.5*sqrt(3)*big_a + distance - s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a + distance - s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + distance + small_a * sqrt(3) / 2 + 2 * s*i };
//
//
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//			}
//		}*/
//
//
//		for (; m <= M; m++) //一
//		{
//			if (m % 2 == 0)
//			{
//				for (int n = 0; n <= N; n++)
//				{
//					point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,             m*0.5*sqrt(3)*big_a + distance - s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a + distance - s * i };
//					tripoint.push_back(B); tripoint.push_back(C);
//				}
//			}
//			else
//			{
//				for (int n = N; n >= 0; n--)
//				{
//					point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,             m*0.5*sqrt(3)*big_a + distance - s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a + distance - s * i };
//					tripoint.push_back(C); tripoint.push_back(B);
//				}
//			}
//		}
//		m = 0, n = 0;
//		for (; n <= N; n++) //丿
//		{
//			if (n % 2 == 0)
//			{
//				for (int m = 0; m <= M; m++)
//				{
//					point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + distance + small_a * sqrt(3) / 2 + 2 * s*i };
//					point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,             m*0.5*sqrt(3)*big_a + distance - s * i };
//					tripoint.push_back(B); tripoint.push_back(A);
//				}
//			}
//			else
//			{
//				for (int m = M; m >= 0; m--)
//				{
//					point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + distance + small_a * sqrt(3) / 2 + 2 * s*i };
//					point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,             m*0.5*sqrt(3)*big_a + distance - s * i };
//					tripoint.push_back(A); tripoint.push_back(B);
//				}
//			}
//		}
//
//		for (int j = 1; j <= 2 * M + 1; j++)//这里是斜着遍历捺
//		{
//			if (j <= M + 1)
//			{
//				if (j % 2 == 0)
//				{
//					n = N - j + 1;
//					m = M;
//					goto even2;
//				}
//				else
//				{
//					n = N;
//					m = M - j + 1;
//					goto odd2;
//				}
//			}
//			else
//			{
//				if (j % 2 == 0)
//				{
//					n = 0;
//					m = 2 * M + 1 - j;
//					goto even2;
//				}
//				else
//				{
//					n = 2 * M + 1 - j;
//					m = 0;
//					goto odd2;
//				}
//			}
//
//		even2:
//			do//奇数行要反捺，和前面的提对应，
//			{
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + distance + small_a * sqrt(3) / 2 + 2 * s*i };
//				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a + distance - s * i };
//				tripoint.push_back(A); tripoint.push_back(C);
//
//				m--;
//				n++;
//			} while (m >= 0 && n <= N);
//			continue;
//		odd2:
//			do
//			{
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + distance + small_a * sqrt(3) / 2 + 2 * s*i };
//				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a + distance - s * i };
//				tripoint.push_back(C); tripoint.push_back(A);
//
//				m++;
//				n--;
//			} while (m <= M && n >= 0);
//
//		}
//		break;
//	}
//
//	case 3://和1相比，横坐标不表，纵坐标取反
//	{
//		i = i - int(2 * distance / s);
//
//		/*for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                     m*0.5*sqrt(3)*big_a - s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a - s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 + 2 * s * i };
//
//
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//			}
//		}*/
//
//
//		for (; m <= M; m++) //一
//		{
//			if (m % 2 == 0)
//			{
//				for (int n = 0; n <= N; n++)
//				{
//					point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                     m*0.5*sqrt(3)*big_a - s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a - s * i };
//					tripoint.push_back(B); tripoint.push_back(C);
//				}
//			}
//			else
//			{
//				for (int n = N; n >= 0; n--)
//				{
//					point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                     m*0.5*sqrt(3)*big_a - s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a - s * i };
//					tripoint.push_back(C); tripoint.push_back(B);
//				}
//			}
//		}
//		m = 0, n = 0;
//		for (; n <= N; n++) //丿
//		{
//			if (n % 2 == 0)
//			{
//				for (int m = 0; m <= M; m++)
//				{
//					point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 + 2 * s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a - s * i };
//					tripoint.push_back(A); tripoint.push_back(C);
//				}
//			}
//			else
//			{
//				for (int m = M; m >= 0; m--)
//				{
//					point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 + 2 * s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a - s * i };
//					tripoint.push_back(C); tripoint.push_back(A);
//				}
//			}
//		}
//
//		for (int j = 1; j <= 2 * M + 1; j++)//这里是斜着遍历捺
//		{
//			if (j <= M + 1)
//			{
//				if (j % 2 == 0)
//				{
//					n = N - j + 1;
//					m = M;
//					goto even3;
//				}
//				else
//				{
//					n = N;
//					m = M - j + 1;
//					goto odd3;
//				}
//			}
//			else
//			{
//				if (j % 2 == 0)
//				{
//					n = 0;
//					m = 2 * M + 1 - j;
//					goto even3;
//				}
//				else
//				{
//					n = 2 * M + 1 - j;
//					m = 0;
//					goto odd3;
//				}
//			}
//
//		even3:
//			do//奇数行要反捺，和前面的提对应，
//			{
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 + 2 * s * i };
//				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                     m*0.5*sqrt(3)*big_a - s * i };
//				tripoint.push_back(B); tripoint.push_back(A);
//
//				m--;
//				n++;
//			} while (m >= 0 && n <= N);
//			continue;
//		odd3:
//			do
//			{
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 + 2 * s * i };
//				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                     m*0.5*sqrt(3)*big_a - s * i };
//				tripoint.push_back(A); tripoint.push_back(B);
//
//				m++;
//				n--;
//			} while (m <= M && n >= 0);
//
//		}
//
//		break;
//	}
//
//	case 4://与2相比，横坐标不变，纵坐标取反
//	{
//		i = i - int(3 * distance / s);
//
//		/*for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a - distance + s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a - distance + s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - distance - small_a * sqrt(3) / 2 - 2 * s*i };
//
//
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//			}
//		}*/
//
//
//		for (; m <= M; m++) //一
//		{
//			if (m % 2 == 0)
//			{
//				for (int n = 0; n <= N; n++)
//				{
//					point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a - distance + s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a - distance + s * i };
//					tripoint.push_back(B); tripoint.push_back(C);
//				}
//			}
//			else
//			{
//				for (int n = N; n >= 0; n--)
//				{
//					point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a - distance + s * i };
//					point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a - distance + s * i };
//					tripoint.push_back(C); tripoint.push_back(B);
//				}
//			}
//		}
//		m = 0, n = 0;
//		for (; n <= N; n++) //丿
//		{
//			if (n % 2 == 0)
//			{
//				for (int m = 0; m <= M; m++)
//				{
//					point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - distance - small_a * sqrt(3) / 2 - 2 * s*i };
//					point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a - distance + s * i };
//					tripoint.push_back(A); tripoint.push_back(C);
//				}
//			}
//			else
//			{
//				for (int m = M; m >= 0; m--)
//				{
//					point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - distance - small_a * sqrt(3) / 2 - 2 * s*i };
//					point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a - distance + s * i };
//					tripoint.push_back(C); tripoint.push_back(A);
//				}
//			}
//		}
//
//		for (int j = 1; j <= 2 * M + 1; j++)//这里是斜着遍历捺
//		{
//			if (j <= M + 1)
//			{
//				if (j % 2 == 0)
//				{
//					n = N - j + 1;
//					m = M;
//					goto even4;
//				}
//				else
//				{
//					n = N;
//					m = M - j + 1;
//					goto odd4;
//				}
//			}
//			else
//			{
//				if (j % 2 == 0)
//				{
//					n = 0;
//					m = 2 * M + 1 - j;
//					goto even4;
//				}
//				else
//				{
//					n = 2 * M + 1 - j;
//					m = 0;
//					goto odd4;
//				}
//			}
//
//		even4:
//			do//奇数行要反捺，和前面的提对应，
//			{
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - distance - small_a * sqrt(3) / 2 - 2 * s*i };
//				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a - distance + s * i };
//				tripoint.push_back(B); tripoint.push_back(A);
//
//				m--;
//				n++;
//			} while (m >= 0 && n <= N);
//			continue;
//		odd4:
//			do
//			{
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - distance - small_a * sqrt(3) / 2 - 2 * s*i };
//				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a - distance + s * i };
//				tripoint.push_back(A); tripoint.push_back(B);
//
//				m++;
//				n--;
//			} while (m <= M && n >= 0);
//
//		}
//		break;
//	}
//
//	default: printf("有问题");
//		
//	}
//}

//void triangle(int i)  //试图贯通所有的洞
//{
//
//	float big_a = 10.0;
//	float small_a = 6.0;
//
//	float s = 0.1;//步长,要考虑填充线的宽度，0<s<=width/2，
//
//	float distance = (big_a - small_a) / (2 * sqrt(3));
//
//	while (i > 6 * distance / s)
//	{
//		i = i - int(6 * distance / s);
//	}
//	int condition;
//	if (i < 1 * distance / s) condition = 1;
//
//	else if (i < 2 * distance / s) condition = 2;
//
//	else if (i < 3 * distance / s) condition = 3;
//
//	else if (i < 4 * distance / s) condition = 4;
//
//	else if (i < 5 * distance / s) condition = 5;
//
//	else if (i < 6 * distance / s) condition = 6;
//
//	switch (condition)
//	{
//	case 1:
//	{
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                       m*0.5*sqrt(3)*big_a + s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a + s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) / 2 - 2 * s * i };
//
//
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//
//
//			}
//		}
//		break;
//	}
//
//	case 2:
//	{
//		i = i - int(distance / s);
//
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a + distance - s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a + distance - s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a + distance + small_a * sqrt(3) * 0.5 + 2 * s*i };
//
//
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//			}
//		}
//		break;
//	}
//
//	case 3://和1相比，横坐标不表，纵坐标取反
//	{
//		i = i - int(2 * distance / s);
//
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + s * sqrt(3)*i  ,                     m*0.5*sqrt(3)*big_a - s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - s * sqrt(3)*i ,              m*0.5*sqrt(3)*big_a - s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 + 2 * s * i };
//
//
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//			}
//		}
//		break;
//	}
//
//	case 4://与2相比，横坐标不变，纵坐标取反
//	{
//		i = i - int(3 * distance / s);
//
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { m*0.5*big_a + n * big_a + 0 ,                                  m*0.5*sqrt(3)*big_a + 0 };
//				point B = { m*0.5*big_a + n * big_a + (distance - s * i) * sqrt(3) ,       m*0.5*sqrt(3)*big_a - distance + s * i };
//
//				point F = { m*0.5*big_a + n * big_a + big_a  ,                             m*0.5*sqrt(3)*big_a + 0 };
//				point C = { m*0.5*big_a + n * big_a + big_a - (distance - s * i) * sqrt(3),m*0.5*sqrt(3)*big_a - distance + s * i };
//
//				point D = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - big_a * sqrt(3) / 2 };
//				point A = { m*0.5*big_a + n * big_a + big_a / 2 ,                          m*0.5*sqrt(3)*big_a - distance - small_a * sqrt(3) / 2 - 2 * s*i };
//
//
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//			}
//		}
//		break;
//	}
//
//	case 5:
//	{
//		i = i - int(4 * distance / s);
//
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { n * big_a + s * sqrt(3)*i ,                     m*sqrt(3)*big_a + sqrt(3)*big_a - s * i };
//				point B = { n * big_a + s * sqrt(3)*i ,                     m*sqrt(3)*big_a + s * i };
//
//				point F = { n * big_a + big_a - s * sqrt(3)*i ,             m*sqrt(3)*big_a + sqrt(3)*big_a - s * i };
//				point C = { n * big_a + big_a - s * sqrt(3)*i ,             m*sqrt(3)*big_a + s * i };
//
//				point D = { n * big_a + big_a * 0.5 ,                       m*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 + 2 * s * i };
//				point A = { n * big_a + big_a * 0.5 ,                       m*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 - 2 * s * i };
//
//
//
//				//point K = { n * big_a + big_a * 0.5 + s * sqrt(3)*i,        m*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 + s * i };
//				//point H = { n * big_a + big_a * 0.5 + s * sqrt(3)*i,        m*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 - s * i };
//
//				//point J = { n * big_a + big_a ,                             m*sqrt(3)*big_a + sqrt(3)*big_a - 2 * s * i };
//				//point G = { n * big_a + big_a ,                             m*sqrt(3)*big_a + 2 * s * i };
//
//				//point L = { n * big_a + big_a * 1.5 - s * sqrt(3)*i ,       m*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 + s * i };
//				//point I = { n * big_a + big_a * 1.5 - s * sqrt(3)*i,        m*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 - s * i };
//
//
//
//
//				tripoint.push_back(D); tripoint.push_back(E);
//				tripoint.push_back(E); tripoint.push_back(F);
//				tripoint.push_back(F); tripoint.push_back(D);
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//
//				/*tripoint.push_back(K); tripoint.push_back(J);
//				tripoint.push_back(J); tripoint.push_back(L);
//				tripoint.push_back(L); tripoint.push_back(K);
//				tripoint.push_back(H); tripoint.push_back(G);
//				tripoint.push_back(G); tripoint.push_back(I);
//				tripoint.push_back(I); tripoint.push_back(H);
//*/
//
//			}
//		}
//		break;
//	}
//
//	case 6:
//	{
//		i = i - int(5 * distance / s);
//
//		for (int m = 0; m < 150 / big_a * sqrt(3) / 2; m++)
//		{
//			for (int n = 0; n < 200 / big_a; n++)
//			{
//				point E = { n * big_a + (distance - s * i) * sqrt(3) ,           m*sqrt(3)*big_a + sqrt(3)*big_a - distance + s * i };
//				point B = { n * big_a + (distance - s * i) * sqrt(3) ,           m*sqrt(3)*big_a + distance - s * i };
//
//				point F = { n * big_a + big_a - (distance - s * i) * sqrt(3) ,   m*sqrt(3)*big_a + sqrt(3)*big_a - distance + s * i };
//				point C = { n * big_a + big_a - (distance - s * i) * sqrt(3) ,   m*sqrt(3)*big_a + distance - s * i };
//
//				point D = { n * big_a + big_a / 2 ,                              m*sqrt(3)*big_a + 5*distance + small_a * sqrt(3) * 0.5 - 2 * s*i };
//				point A = { n * big_a + big_a / 2 ,                              m*sqrt(3)*big_a + distance + small_a * sqrt(3) * 0.5 + 2 * s*i };
//
//				/*point K = { n * big_a + big_a - 0.5*small_a - s * i * sqrt(3),   m*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 +distance - s * i };
//				point H = { n * big_a + big_a - 0.5*small_a - s * i * sqrt(3),   m*sqrt(3)*big_a + small_a * sqrt(3) * 0.5 + 2*distance + s * i };
//
//				point J = { n * big_a + big_a ,                                  m*sqrt(3)*big_a + sqrt(3)*big_a - 2 * distance + 2 * s * i };
//				point G = { n * big_a + big_a ,                                  m*sqrt(3)*big_a + 2 * distance - 2 * s * i };
//
//				point L = { n * big_a + big_a + 0.5*small_a + s * sqrt(3)*i,     m*sqrt(3)*big_a + big_a * sqrt(3) * 0.5 + distance - s * i };
//				point I = { n * big_a + big_a + 0.5*small_a + s * sqrt(3)*i,     m*sqrt(3)*big_a + small_a * sqrt(3) * 0.5 + 2 * distance + s * i };
//*/
//
//				tripoint.push_back(D); tripoint.push_back(E);
//				tripoint.push_back(E); tripoint.push_back(F);
//				tripoint.push_back(F); tripoint.push_back(D);
//				tripoint.push_back(A); tripoint.push_back(B);
//				tripoint.push_back(B); tripoint.push_back(C);
//				tripoint.push_back(C); tripoint.push_back(A);
//
//				/*tripoint.push_back(K); tripoint.push_back(J);
//				tripoint.push_back(J); tripoint.push_back(L);
//				tripoint.push_back(L); tripoint.push_back(K);
//				tripoint.push_back(H); tripoint.push_back(G);
//				tripoint.push_back(G); tripoint.push_back(I);
//				tripoint.push_back(I); tripoint.push_back(H);*/
//
//			}
//		}
//		break;
//	}
//
//	default: printf("有问题");
//		break;
//	}
//}

void intersection()           //在这里面调节要不要变化
{
	for (int h = 0; h < shell1.size(); h++)
	{
		for (int i = 0; i < shell1[h].size(); i++)
		{
			tripoint.clear(); //把上一层的变化线清理掉
			interpoint.clear(); //把上一层的交点清理掉
			polypoint.clear();//把上一层的轮廓线清理掉
			triangle(h); //产生第i层的变化线存在tripoint        0：不变三角，48不变六边形， i变化线

			polypoint = shell1[h][i];//把第i层的轮廓线给polypoint

			for (int m = 0; m < tripoint.size(); m += 2)  //网格的每一条线段，首尾相连线段端点重复，所以跳两个
			{
				interpoint.clear(); //上一条线段的交点清理掉
				for (int n = 0; n < polypoint.size(); n++) //多边形的每一条线段，首尾相连线段端点不重复，所以跳一个
				{
					inter(tripoint[m], tripoint[m + 1], polypoint[n], polypoint[(n + 1 + polypoint.size()) % polypoint.size()]);
				}
				int a = InOrOutPolygon(tripoint[m]);
				int b = InOrOutPolygon(tripoint[m + 1]);  //判断是不是内点,0:外 1:内

				if (interpoint.size() == 0)   // 无交点
				{
					if ((a == 0) && (b == 0)) //无内点，舍
					{
						continue;
					}
					if ((a == 1) || (b == 1)) //存在内点，取
					{
						new_tripoint.push_back(tripoint[m]);
						new_tripoint.push_back(tripoint[m + 1]);
						continue;
					}
				}

				if (interpoint.size() == 1)   //有一交点
				{
					if ((a == 0) && (b == 0))   //有一交点且无内点，舍 （相切）
					{
						continue;
					}
					if ((a == 1) && (b == 0))//有一交点且有一内点，取内点到交点 （相交）
					{
						new_tripoint.push_back(tripoint[m]);
						new_tripoint.push_back(interpoint[0]);
						continue;
					}
					if ((a == 0) && (b == 1))//有一交点且有一内点，取内点到交点 （相交）
					{
						new_tripoint.push_back(interpoint[0]);
						new_tripoint.push_back(tripoint[m + 1]);
						continue;
					}
					if ((a == 1) && (b == 1)) //有一交点且有两内点，取 （内部包含）
					{
						new_tripoint.push_back(tripoint[m]);
						new_tripoint.push_back(tripoint[m + 1]);
						continue;
					}

				}

				if (interpoint.size() == 2)   //有两交点
				{
					if ((a == 0) && (b == 0))   //有两交点且无内点，取两交点
					{
						new_tripoint.push_back(interpoint[0]);
						new_tripoint.push_back(interpoint[1]);
						continue;
					}
					if ((a == 1) && (b == 0))//有两交点且一个内点 两两相连，注意顺序 
					{
						new_tripoint.push_back(tripoint[m]);
						new_tripoint.push_back(mindistance(tripoint[m], interpoint[0], interpoint[1]));
						continue;
					}
					if ((a == 0) && (b == 1))//有两交点且一个内点 两两相连，注意顺序
					{
						new_tripoint.push_back(tripoint[m + 1]);
						new_tripoint.push_back(mindistance(tripoint[m + 1], interpoint[0], interpoint[1]));
						continue;
					}
					if ((a == 1) && (b == 1))//有两交点且两个内点 两两相连，注意顺序 
					{
						new_tripoint.push_back(tripoint[m]);
						new_tripoint.push_back(mindistance(tripoint[m], interpoint[0], interpoint[1]));
						new_tripoint.push_back(mindistance(tripoint[m + 1], interpoint[0], interpoint[1]));
						new_tripoint.push_back(tripoint[m + 1]);
						continue;
					}

				}

			}

			modelfill.push_back(new_tripoint);              //存该轮廓的点
			new_tripoint.clear();
			
		}
		printf("%.2lf%%\r", h * 100.0 / model.size());
		modfill.push_back(modelfill);
		modelfill.clear();
	}
}


void writegcode_ultimaker3(float z) //cura
{
	float t = 0.017776;//对应0.2的层厚,宽度0.42
	double E = 0;

	FILE* fp;

	errno_t err;     //判断此文件流是否存在 存在返回1

	err = fopen_s(&fp, "cura.gcode", "a"); //若return 1 , 则将指向这个文件的文件流给fp1

	fprintf(fp, "T0\n");
	fprintf(fp, "M82\n");
	fprintf(fp, "G92 E0\n");
	fprintf(fp, "M109 S210\n");
	fprintf(fp, "G280 S1\n");
	fprintf(fp, "G0 Z20.001\n");
	fprintf(fp, "G1 F1500 E-6.5\n");
	fprintf(fp, "M107\n");
	fprintf(fp, ";LAYER_COUNT: %d\n", modelfill.size());

	fprintf(fp, ";LAYER:0\n");

	fprintf(fp, "G0 F2000 Z%.1f\n", 0.1);
	for (int i = 0; Bmin_x + 2 * i * 0.42 + 0.42 < Bmax_x; i++)  //第1、2层打实，第一层高度是0.27
	{
		fprintf(fp, "G0 F2000 X%f Y%f \n", Bmin_x + 2 * i * 0.42, Bmin_y);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmin_x + 2 * i * 0.42, Bmax_y, E += (Bmax_y - Bmin_y) * t);
		fprintf(fp, "G0 F2000 X%f Y%f \n", Bmin_x + 2 * i * 0.42 + 0.42, Bmax_y);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmin_x + 2 * i * 0.42 + 0.42, Bmin_y, E += (Bmax_y - Bmin_y) * t);
	}
	fprintf(fp, ";LAYER:1\n");

	fprintf(fp, "G0 F2000 Z%.1f\n", 0.3);
	for (int i = 0; Bmin_y + 2 * i * 0.42 + 0.42 < Bmax_y; i++)  //这是第二层，二层以后都是0.2
	{
		fprintf(fp, "G0 F2000 X%f Y%f \n", Bmin_x, Bmin_y + 2 * i * 0.42);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmax_x, Bmin_y + 2 * i * 0.42, E += (Bmax_y - Bmin_y) * t);
		fprintf(fp, "G0 F2000 X%f Y%f \n", Bmax_x, Bmin_y + 2 * i * 0.42 + 0.42);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmin_x, Bmin_y + 2 * i * 0.42 + 0.42, E += (Bmax_y - Bmin_y) * t);
	}


	for (int i = 0; i < model.size() - 4; i++) //i代表层数，要加几层打实在的，界限就减几，再改高度   
	{
		fprintf(fp, ";LAYER:%d\n", i + 2);//加了几层底，这里就加几
		fprintf(fp, ";TYPE:OUTLINE\n");
		//fprintf(fp, "G0 F2000 Z%.3f\n", 0.500 + i * z);//去掉轮廓线时用到，确定Z


		fprintf(fp, "M73 P%.f\n", float(  (100*i)/model.size() ) );
		for (int j = 0; j < shell1[i].size(); j++)
			//轮廓线
		{
			//打印最内层
			fprintf(fp, "G0 F2000 X%.3f Y%.3f Z%.1f\n", shell1[i][j][0].x, shell1[i][j][0].y, 0.3 + i * z);//3层高度加i*z，i从0开始

			fprintf(fp, "G1 F1200 X%.3f Y%.3f E%.5f\n", shell1[i][j][1].x, shell1[i][j][1].y, E += distance(shell1[i][j][0], shell1[i][j][1]) * t);

			for (int k = 2; k < shell1[i][j].size(); k++)

			{
					fprintf(fp, "G1 X%.3f Y%.3f E%.5f\n", shell1[i][j][k].x, shell1[i][j][k].y, E += distance(shell1[i][j][k-1], shell1[i][j][k]) * t);

			}
			//fprintf(fp, "G1 X%.3f Y%.3f E%.5f\n", model[i][0].x, model[i][0].y, E += distance(model[i][model[i].size() - 1], model[i][0]) * t); //画一个圈，要回原点
			
			//打印外壳
			for (int k = 0; k < shell[i][j].size(); k++)
			{
				fprintf(fp, "G0 F2000 X%.3f Y%.3f Z%.1f\n", shell[i][j][k][0].x, shell[i][j][k][0].y, 0.3 + i * z);//3层高度加i*z，i从0开始

				fprintf(fp, "G1 F1200 X%.3f Y%.3f E%.5f\n", shell[i][j][k][1].x, shell[i][j][k][1].y, E += distance(shell[i][j][k][0], shell[i][j][k][1]) * t);

				for (int m = 2; m < shell[i][j][k].size(); m++)

				{
					fprintf(fp, "G1 X%.3f Y%.3f E%.5f\n", shell[i][j][k][m].x, shell[i][j][k][m].y, E += distance(shell[i][j][k][m - 1], shell[i][j][k][m]) * t);

				}
			}
			//fprintf(fp, "G1 X%.3f Y%.3f E%.5f\n", model[i][1].x, model[i][1].y, E += distance(model[i][model[i].size() - 1], model[i][0]) * 0.01);//回到原点再多出去一点，有利于不翘边
		}
		//填充线
		fprintf(fp, ";TYPE:FILL\n");
		for(int j=0;j<modfill[i].size();j++)
		{ 		
			for (int k = 1; k < modfill[i][j].size(); k += 2)
			{
			fprintf(fp, "G0 F2000 X%.3f Y%.3f\n", modfill[i][j][k - 1].x, modfill[i][j][k - 1].y);
			fprintf(fp, "G1 F1200 X%.3f Y%.3f E%.5f\n", modfill[i][j][k].x, modfill[i][j][k].y, E += distance(modfill[i][j][k - 1], modfill[i][j][k]) * t);
			}
		}

	}

	/*fprintf(fp, ";LAYER:%d\n", model.size() - 2);

	fprintf(fp, "G0 F2000 Z%.1f\n", 0.3 + (model.size() - 3) * z);
	for (int i = 0; Bmin_x + 2 * i * 0.42 + 0.42 < Bmax_x; i++)  //倒数两层打实
	{
		fprintf(fp, "G0 F2000 X%f Y%f \n", Bmin_x + 2 * i * 0.42, Bmin_y);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmin_x + 2 * i * 0.42, Bmax_y, E += (Bmax_y - Bmin_y) * t);
		fprintf(fp, "G0 F2000 X%f Y%f \n", Bmin_x + 2 * i * 0.42 + 0.42, Bmax_y);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmin_x + 2 * i * 0.42 + 0.42, Bmin_y, E += (Bmax_y - Bmin_y) * t);
	}
	fprintf(fp, ";LAYER:%d\n", model.size() - 1);

	fprintf(fp, "G0 F2000 Z%.1f\n", 0.3 + (model.size() - 2) * z);
	for (int i = 0; Bmin_y + 2 * i * 0.42 + 0.42 < Bmax_y; i++)
	{
		fprintf(fp, "G0 F2000 X%f Y%f Z%.1f\n", Bmin_x, Bmin_y + 2 * i * 0.42, 0.47 + (model.size() - 2) * z);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmax_x, Bmin_y + 2 * i * 0.42, E += (Bmax_y - Bmin_y) * t);
		fprintf(fp, "G0 F2000 X%f Y%f \n", Bmax_x, Bmin_y + 2 * i * 0.42 + 0.42);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmin_x, Bmin_y + 2 * i * 0.42 + 0.42, E += (Bmax_y - Bmin_y) * t);
	}*/

	fprintf(fp, "G1 F1500 E%f\n", E -= 6);
	fprintf(fp, "M104 S0                     ;extruder heater off\n");
	fprintf(fp, "M107\n");
	fprintf(fp, "G91\n");
	fprintf(fp, "G90                         ;Disable relative movement\n");
	fprintf(fp, "M82                         ;absolute extrusion mode\n");
	fprintf(fp, "M104 S0\n");
	fprintf(fp, "M104 T1 S0\n");
	fprintf(fp, ";End GCode\n");

	fclose(fp);
}
void writegcode(float z) //盘纹的格式，或者cura
{
	float t = 0.060;//对应0.2的层厚
	double E = 0;

	FILE* fp;

	errno_t err;     //判断此文件流是否存在 存在返回1

	err = fopen_s(&fp, "cura.gcode", "a"); //若return 1 , 则将指向这个文件的文件流给fp1

	fprintf(fp, "M104 S190\n");
	fprintf(fp, "M105\n");
	fprintf(fp, "M109 S200\n");
	fprintf(fp, "M82;set extruder to absolute mode\n");
	fprintf(fp, "G28;move X/Y to min endstops\n");
	fprintf(fp, "G1 Z15.0 F6000 ;move the platform down 15mm\n");
	fprintf(fp, "G92 E0                  ;zero the extruded length\n");
	fprintf(fp, "G1 F200 E3             ;extrude 3mm of feed stock\n");
	fprintf(fp, "G92 E0\n");
	fprintf(fp, "G92 E0\n");
	fprintf(fp, "G1 F1500 E-6.5\n");

	for (int i = 0; Bmin_x + i * 4 < Bmax_x; i++)  //底座
	{
		fprintf(fp, "G1 F2000 X%f Y%f Z%f F600\n", Bmin_x + i * 4, Bmin_y, 0.3);
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmin_x + i * 4, Bmax_y, E += (Bmax_y - Bmin_y)*(t + 0.03));
		fprintf(fp, "G1 F300  X%f Y%f E%f\n", Bmin_x + i * 4 + 2, Bmax_y, E += (2)*(t + 0.03));
		fprintf(fp, "G1 F1000 X%f Y%f E%f\n", Bmin_x + i * 4 + 2, Bmin_y, E += (Bmax_y - Bmin_y)*(t + 0.03));
		fprintf(fp, "G1 F300  X%f Y%f E%f\n", Bmin_x + i * 4 + 4, Bmin_y, E += (2)*(t + 0.03));
	}

	fprintf(fp, ";LAYER_COUNT: %d\n", modelfill.size() + 1);
	for (int i = 1; i < model.size(); i++) //每一层 i代表层数      z层厚 t挤出率
	{
		fprintf(fp, ";LAYER:%d\n", i);

		fprintf(fp, "G0 F2000 Z%.3f\n", 0.500 + i * z);//去掉轮廓线时用到，确定Z

		//轮廓线
		fprintf(fp, "G0 F2000 X%.3f Y%.3f Z%.3f\n", model[i][0].x, model[i][0].y, 0.500 + i * z);
		fprintf(fp, ";TYPE:OUTLINE\n");
		fprintf(fp, "G1 F1200 X%.3f Y%.3f E%.5f\n", model[i][1].x, model[i][1].y, E += distance(model[i][0], model[i][1])*t);
		for (int j = 2; j < model[i].size(); j++)
		{
			fprintf(fp, "G1 X%.3f Y%.3f E%.5f\n", model[i][j].x, model[i][j].y, E += distance(model[i][j - 1], model[i][j])*t);
		}
		fprintf(fp, "G1 X%.3f Y%.3f E%.5f\n", model[i][0].x, model[i][0].y, E += distance(model[i][model[i].size() - 1], model[i][0])*t); //画一个圈，要回原点
		fprintf(fp, "G1 X%.3f Y%.3f E%.5f\n", model[i][1].x, model[i][1].y, E += distance(model[i][model[i].size() - 1], model[i][0])*0.01);//回到原点再多出去一点，有利于不翘边

		//填充线
		fprintf(fp, ";TYPE:FILL\n");
		for (int k = 1; k < modelfill[i].size(); k += 2)
		{
			fprintf(fp, "G0 F2000 X%.3f Y%.3f\n", modelfill[i][k - 1].x, modelfill[i][k - 1].y);
			fprintf(fp, "G1 F1200 X%.3f Y%.3f E%.5f\n", modelfill[i][k].x, modelfill[i][k].y, E += distance(modelfill[i][k - 1], modelfill[i][k])*t);
		}
	}

	fprintf(fp, "M107\n");
	fprintf(fp, "M104 S0                     ;extruder heater off\n");
	fprintf(fp, "G92 E1                      ;relative positioning\n");
	fprintf(fp, "G1 E-1 F300                 ;retract the filament a bit before lifting the nozzle, to release some of the pressure\n");
	fprintf(fp, "G1 X120 Y120 Z120 F9000     ;move to max so the object can take out\n");
	fprintf(fp, "M84                         ;steppers off\n");
	fprintf(fp, "M82                         ;absolute extrusion mode\n");
	fprintf(fp, "M104 S0\n");
	fprintf(fp, ";End GCode\n");

	fclose(fp);
}

void writegcode_makerbot(float z)//还是用cura的格式然后用x3g转换吧
{
	int layers = model.size();//打印层数
	float layer_height = z;//层高,改此参数要改步进率t
	float t = 0.0349595312;
	//if(layer_height == 0.2)      t = 0.0349595312;//步进率层高为0.2
	//else if(layer_height == 0.3) t = 0.0522797843;//步进率层高为0.3
	int F = 1800;//打印速度 mm/min

	double A = 0.0;//步进机

	FILE* fp;

	errno_t err;     //判断此文件流是否存在 存在返回1

	err = fopen_s(&fp, "makerbot.gcode", "a"); //若return 1 , 则将指向这个文件的文件流给fp1

	//第一层慢一点
	fprintf(fp, "M73 P%d\n", 0);//打印进度百分比
	fprintf(fp, "G1 X%f Y%f Z%f F3000;travel move\n", Bmin_x - 0.01, Bmin_y - 0.01, 0.3);
	//for (int i = 0; Bmin_x+i*4<Bmax_x; i++)  //底座
	//{
	//	fprintf(fp, "G1 X%f Y%f Z%f F600\n",     Bmin_x + i * 4,   Bmin_y, 0.3);
	//	fprintf(fp, "G1 X%f Y%f Z%f F600 A%f\n", Bmin_x + i * 4,   Bmax_y, 0.3, A += (Bmax_y- Bmin_y)*(t+0.03));
	//	fprintf(fp, "G1 X%f Y%f Z%f F600 A%f\n", Bmin_x + i * 4+2, Bmax_y, 0.3, A += (2)*(t+0.03));
	//	fprintf(fp, "G1 X%f Y%f Z%f F600 A%f\n", Bmin_x + i * 4+2, Bmin_y, 0.3, A += (Bmax_y - Bmin_y)*(t+0.03));
	//	fprintf(fp, "G1 X%f Y%f Z%f F600 A%f\n", Bmin_x + i * 4+4, Bmin_y, 0.3, A += (2)*(t+0.03));
	//}
	fprintf(fp, ";LAYER:0\n");
	for (int i = 0; Bmin_x + 2 * i * 0.42 + 0.42 < Bmax_x; i++)  //第1、2层打实，第一层高度是0.27
	{
		fprintf(fp, "G0 F600 X%f Y%f Z%f\n", Bmin_x + 2 * i * 0.42, Bmin_y, 0.27);
		fprintf(fp, "G1 F600 X%f Y%f Z%f A%f\n", Bmin_x + 2 * i * 0.42, Bmax_y, 0.27,A += (Bmax_y - Bmin_y)*t);
		fprintf(fp, "G0 F600 X%f Y%f Z%f\n", Bmin_x + 2 * i * 0.42 + 0.42, Bmax_y, 0.27);
		fprintf(fp, "G1 F600 X%f Y%f Z%f A%f\n", Bmin_x + 2 * i * 0.42 + 0.42, Bmin_y, 0.27,A += (Bmax_y - Bmin_y)*t);
	}
	fprintf(fp, ";LAYER:1\n");
	for (int i = 0; Bmin_y + 2 * i * 0.42 + 0.42 < Bmax_y; i++)  //这是第二层，二层以后都是0.2
	{
		fprintf(fp, "G0 F600 X%f Y%f Z%f\n", Bmin_x, Bmin_y + 2 * i * 0.42, 0.47);
		fprintf(fp, "G1 F600 X%f Y%f Z%f A%f\n", Bmax_x, Bmin_y + 2 * i * 0.42, 0.47,A += (Bmax_y - Bmin_y)*t);
		fprintf(fp, "G0 F600 X%f Y%f Z%f\n", Bmax_x, Bmin_y + 2 * i * 0.42 + 0.42, 0.47);
		fprintf(fp, "G1 F600 X%f Y%f Z%f A%f\n", Bmin_x, Bmin_y + 2 * i * 0.42 + 0.42, 0.47,A += (Bmax_y - Bmin_y)*t);
	}

	for (int i = 0; i < model.size()-4; i++) //打印轮廓和填充,打印几层实的就减几，再改高度
	{
		fprintf(fp, ";LAYER:%d\n",i+2);
		fprintf(fp, "M73 P%d\n", int(i / layers) * 100);//打印进度百分比
		fprintf(fp, "G0 X%f Y%f Z%f F2000;travel move\n", modelfill[i][0].x-0.01, modelfill[i][0].y-0.01, 0.67 + i * z);

		fprintf(fp, "G0 F2000 X%.3f Y%.3f Z%.3f\n", model[i][0].x, model[i][0].y, 0.67 + i * z);
		fprintf(fp, ";TYPE:OUTLINE\n");
		fprintf(fp, "G1 F1200 X%.3f Y%.3f Z%f A%.5f\n", model[i][1].x, model[i][1].y, 0.67 + i * z, A += distance(model[i][0], model[i][1])*t);
		for (int j = 2; j < model[i].size(); j++)
		{
			fprintf(fp, "G1 X%.3f Y%.3f Z%f A%.5f\n", model[i][j].x, model[i][j].y, 0.67 + i * z, A += distance(model[i][j - 1], model[i][j])*t);
		}
		fprintf(fp, "G1 X%.3f Y%.3f Z%f A%.5f\n", model[i][0].x, model[i][0].y, 0.67 + i * z, A += distance(model[i][model[i].size() - 1], model[i][0])*t); //画一个圈，要回原点
		fprintf(fp, "G1 X%.3f Y%.3f Z%f A%.5f\n", model[i][1].x, model[i][1].y, 0.67 + i * z, A += distance(model[i][model[i].size() - 1], model[i][0])*0.01);//回到原点再多出去一点，有利于不翘边

		fprintf(fp, ";TYPE:FILL\n");
		for (int k = 1; k < modelfill[i].size(); k += 2)
		{
			fprintf(fp, "G1 X%f Y%f Z%f F2000\n", modelfill[i][k - 1].x, modelfill[i][k - 1].y, 0.67 + i * z);
			fprintf(fp, "G1 X%f Y%f Z%f F1800 A%f\n", modelfill[i][k].x, modelfill[i][k].y, 0.67 + i * z, A += distance(modelfill[i][k - 1], modelfill[i][k])*t);
		}
	}

	//for (int i = 0; Bmin_x + 2 * i * 0.42 + 0.42 < Bmax_x; i++)  //倒数两层打实
	//{
	//	fprintf(fp, "G0 X%f Y%f Z%f F2000\n", Bmin_x + 2 * i * 0.42, Bmin_y, 0.47 + (model.size() - 3) * z);
	//	fprintf(fp, "G1 X%f Y%f Z%f A%f F1000\n", Bmin_x + 2 * i * 0.42, Bmax_y, 0.47 + (model.size() - 3) * z, A += (Bmax_y - Bmin_y)*t);
	//	fprintf(fp, "G0 X%f Y%f Z%f F2000\n", Bmin_x + 2 * i * 0.42 + 0.42, Bmax_y,0.47 + (model.size() - 3) * z);
	//	fprintf(fp, "G1 X%f Y%f Z%f A%f F1000\n", Bmin_x + 2 * i * 0.42 + 0.42, Bmin_y, 0.47 + (model.size() - 3) * z, A += (Bmax_y - Bmin_y)*t);
	//}
	//fprintf(fp, ";LAYER:%d\n", model.size() - 1);
	//for (int i = 0; Bmin_y + 2 * i * 0.42 + 0.42 < Bmax_y; i++)
	//{
	//	fprintf(fp, "G0 X%f Y%f Z%f F2000\n", Bmin_x, Bmin_y + 2 * i * 0.42, 0.47 + (model.size() - 2) * z);
	//	fprintf(fp, "G1 X%f Y%f Z%f A%f F1000 \n", Bmax_x, Bmin_y + 2 * i * 0.42, 0.47 + (model.size() - 2) * z, A += (Bmax_y - Bmin_y)*t);
	//	fprintf(fp, "G0 X%f Y%f Z%f F2000\n", Bmax_x, Bmin_y + 2 * i * 0.42 + 0.42, 0.47 + (model.size() - 2) * z);
	//	fprintf(fp, "G1 X%f Y%f Z%f A%f F1000 \n", Bmin_x, Bmin_y + 2 * i * 0.42 + 0.42, 0.47 + (model.size() - 2) * z, A += (Bmax_y - Bmin_y)*t);
	//}
	fclose(fp);
}

void InitGL(GLvoid)
{
	glShadeModel(GL_SMOOTH);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

void lines() //画线的函数
{
	//glColor3f(1.0, 0.0, 0.0);
	//glBegin(GL_LINES);
	//for (int j = 0; j < 50; j++)
	//{
	//	for (int i = 0; i < tripoint.size(); i += 2)
	//	{
	//		glVertex3f(tripoint[i].x, tripoint[i].y, 0);  glVertex3f(tripoint[i + 1].x, tripoint[i + 1].y, 0);
	//	}
	//}
	//glEnd();
	glBegin(GL_LINES);//坐标轴
	glColor3f(1.0, 0.0, 0.0);  glVertex3f(0, 0, 0); glVertex3f(100, 0, 0);
	glColor3f(0.0, 1.0, 0.0); glVertex3f(0, 0, 0); glVertex3f(0, 100, 0);
	glColor3f(0.0, 0.0, 1.0); glVertex3f(0, 0, 0); glVertex3f(0, 0, 100);
	glEnd();

	glColor3f(0.945, 0.945, 0.945);
	glBegin(GL_LINES);
	for (int i = 1; i < shell1.size(); i++)
	{
		for (int j = 0; j < shell1[i].size(); j++)
		{
			for(int k = 1;k<shell1[i][j].size();k++)
			{
				glVertex3f(shell1[i][j][k - 1].x, shell1[i][j][k - 1].y, i * 0.2); glVertex3f(shell1[i][j][k].x, shell1[i][j][k].y, i * 0.2);
			}
			glVertex3f(shell1[i][j][0].x, shell1[i][j][0].y, i * 0.2); glVertex3f(shell1[i][j][shell1[i][j].size() - 1].x, shell1[i][j][shell1[i][j].size() - 1].y, i * 0.2);
		}
		
	}
	glEnd();

	glColor3f(0.615, 0.615, 0.615);
	glBegin(GL_LINES);
	for (int i = 1; i < modfill.size(); i += 19)
	{
		for (int j = 0; j < modfill[i].size(); j ++)
		{
			for (int k = 1; k < modfill[i][j].size(); k+=2)
			{
				glVertex3f(modfill[i][j][k - 1].x, modfill[i][j][k - 1].y, i * 0.2); glVertex3f(modfill[i][j][k].x, modfill[i][j][k].y, i * 0.2);
			}
		//	glVertex3f(modelfill[i][j - 1].x, modelfill[i][j - 1].y, i*0.2); glVertex3f(modelfill[i][j].x, modelfill[i][j].y, i*0.2);
		}
	}
	glEnd();

}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glPushMatrix();

	gluLookAt(30 + xs, -30 + ys, 100 + zs, 30 + sx, 30 + sy, 30 + sz, 0, 0, 1);
	//glTranslatef(0.0f, 0.0f, -1.0f);	//平移
	//glRotatef(rquad, 1.0f, 0.0f, 0.0f);	//旋转一个角度
	lines();

	//glColor3f(1.0, 0.0, 0.0);
	//glPointSize(5);
	//glColor3f(1.0,0.0,0.0);
	//glBegin(GL_POINTS);
	////glVertex3f(0, 0, 0);
	////glVertex3f(-50, 0, 1);
	//glVertex3f(60,60,60);
	//glEnd();
	glPopMatrix();

	//rquad -= 0.15f;	//修改立方体的旋转角度
	glutSwapBuffers();
}

void reshape(int width, int height)
{
	if (height == 0)
		height = 1;
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//gluPerspective(90.0f, (GLfloat)width / (GLfloat)height, 0.0f, 100.0f);
	gluPerspective(90.0f, (GLfloat)width / (GLfloat)height, 0.0f, 100.0f);
	//if (width <= height)
	//	glOrtho(-2.0, 2.0, -2.0*(GLfloat)height / (GLfloat)width, 2.0*(GLfloat)height / (GLfloat)width, 1.0, 20.0);
	//else
	//	glOrtho(-2.0*(GLfloat)width / (GLfloat)height, 2.0*(GLfloat)width / (GLfloat)height, -2.0, 2.0, 1.0, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void myMouse(int button, int state, int x, int y)
{
	//if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) //鼠标左键按下
	//{
	//	mousetate = 1;
	//	Oldx = x;
	//	Oldy = y;
	//}
	//if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) //鼠标左键松开
	//{
	//	mousetate = 0;
	//}
	//滚轮事件缩小
	if (state == GLUT_UP && button == 3) {

		scale -= 0.1f;
	}//放大
	if (state == GLUT_UP && button == 4) {

		scale += 0.1f;
	}
	glutPostRedisplay();//标记重绘该界面
}

void onMouseMove(int x, int y)//后续完善
{

}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'w':
		sy += 5;
		glutPostRedisplay();
		break;
	case 's':
		sy -= 5;
		glutPostRedisplay();
		break;
	case 'a':
		sx -= 5;
		glutPostRedisplay();
		break;
	case 'd':
		sx += 5;
		glutPostRedisplay();
		break;
	case 'q':
		sz += 5;
		glutPostRedisplay();
		break;
	case 'e':
		sz -= 5;
		glutPostRedisplay();
		break;
	default:
		break;
	}
}

void SpecialKey(GLint key, GLint x, GLint y)
{
	if (key == GLUT_KEY_UP)
	{
		ys += 10;
	}
	if (key == GLUT_KEY_LEFT)
	{
		xs -= 10;
	}
	if (key == GLUT_KEY_DOWN)
	{
		ys -= 10;
	}
	if (key == GLUT_KEY_RIGHT)
	{
		xs += 10;
	}
	glutPostRedisplay();
}





void print()
{
	for (int i = 0; i < model.size(); i++)
	{   
		cout << "layer  :"<<i<<endl;
		for (int j = 0; j < model[i].size(); j++)
		{
			cout << "(" << model[i][j].x << "," << model[i][j].y << ") ";
		 }
		cout << endl;
	}
}

void main(int argc, char** argv)
{
	float z = 0.2;
	//************************************************************************切片
	printf("slicing...\n");
	readfile(file_1);
	BoundingBox();
	findIntersect(z);  //前三个函数切片，产生model
	//print();
	clipper();//划分截面和偏移
	printf("slice complete,layer count: %d\n", countt);

	//************************************************************************取交
	printf("path planning...\n");
	intersection();
	printf("path planning complete\n"); //modelfill完成

	//************************************************************************编写Gcode
	printf("Gcode writing...\n");
	//writegcode(z);
	//writegcode_makerbot(z);
	writegcode_ultimaker3(z);
	//writpointcloud();
	printf("Gcode writing complete,file save as \"test gcode.gcode\"\n");

	

	
    

	//************************************************************************预览
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(600, 600);
	glutCreateWindow("Hello Cube");
	InitGL();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(display);
	glutMouseFunc(myMouse);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(&SpecialKey);
	glutMainLoop();

}


