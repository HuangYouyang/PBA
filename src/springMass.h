#pragma once
#include <glad/glad.h> 
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "mesh.h"
#include "model.h"
#include "cyTriMesh.h"
#include "cyCore.h"
#include "cyMatrix.h"
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/SparseCholesky"

using namespace std;
using namespace Eigen;

float vectorDotfloatMS(glm::vec3 a, glm::vec3 b)
{
	glm::mat3 r0i = glm::mat3(a, glm::vec3(0.0f), glm::vec3(0.0f));
	glm::mat3 r0i_T = glm::mat3(glm::vec3(b.x, 0, 0), glm::vec3(b.y, 0, 0), glm::vec3(b.z, 0, 0));
	return (r0i_T * r0i)[0][0];
}

glm::mat3 vectorDotMatMS(glm::vec3 a, glm::vec3 b)
{
	glm::mat3 r0i = glm::mat3(a, glm::vec3(0.0f), glm::vec3(0.0f));
	glm::mat3 r0i_T = glm::mat3(glm::vec3(b.x, 0, 0), glm::vec3(b.y, 0, 0), glm::vec3(b.z, 0, 0));
	return r0i * r0i_T;
}

class MassSpring
{
public:
	// model info
	vector<glm::vec3> x;  // position
	vector<glm::vec3> v;  // velocity
	vector<glm::vec2> neighbor;
	vector<float> lengthN;
	float length = 100.0f;

	// force
	vector<glm::vec3> f;
	//vector<glm::mat3>& He;
	float k = 100;
	float t = 0.1;
	float mass = 0.1;
	float damping = 0.1;

	// lock
	int Lock_Index1 = 0;
	int Lock_Index2 = 20;

	void getNeighborList(map<tuple<int, int>, int> neighboringEdges)
	{
		map<tuple<int, int>, int>::iterator iter;
		for (iter = neighboringEdges.begin(); iter != neighboringEdges.end(); iter++)
		{
			tuple<int, int> nodeIndex = iter->first;
			int i = get<0>(nodeIndex) - 1;
			int j = get<1>(nodeIndex) - 1;
			glm::vec3 xij = x[i] - x[j];
			neighbor.push_back(glm::vec2(get<0>(nodeIndex) - 1, get<1>(nodeIndex) - 1));
			lengthN.push_back(glm::length(x[i] - x[j]));
		}
		cout << neighbor.size() << endl;
	}

	void getVelocityandForceList(int num)
	{
		for (int i = 0; i < num; i++)
		{
			this->v.push_back(glm::vec3(0.0));
			if (i > 0)
				this->f.push_back(glm::vec3(0.0, -9.8f, 0.0f));
			else
				this->f.push_back(glm::vec3(0.0f));
		}
	}

	MassSpring()
	{
		this->k = 1;
		this->t = 0.1;
		this->mass = 1;
		this->damping = 0.1;
		this->length = 1.0f;
	}

	void MassSpringInitiate(int Lock_Index1,int Lock_Index2, vector<glm::vec3> vertices, map<tuple<int, int>, int> neighboringEdges)
	{
		this->Lock_Index1 = Lock_Index1;
		this->Lock_Index2 = Lock_Index2;
		this->x = vertices;
		getNeighborList(neighboringEdges);
	    getVelocityandForceList(vertices.size());

		this->k = 1;
		this->t = 0.1;
		this->mass = 1;
		this->damping = 0.1;
		this->length = 1.0f;
	}

	void getGradient(vector<glm::vec3> &x, vector<glm::vec3> &xHat, float t, vector<glm::vec3> &G)
	{
		// calculate first graident
		for (int i = 0; i < x.size(); i++)
		{
			if (i == Lock_Index1 || i == Lock_Index2)
			{
				continue;
			};
			G[i] = 1 / (t * t) * mass * (x[i] - xHat[i]);
		}

		//// calculate force
		//for (int e = 0; e < neighbor.size(); e++)
		//{
		//	glm::vec3 i = x[neighbor[e][0]];
		//	glm::vec3 j = x[neighbor[e][1]];

		//	float normIJ = glm::sqrt((i - j).x * (i - j).x + (i - j).y * (i - j).y + (i - j).z * (i - j).z);

		//	glm::vec3 fNode = -k * (normIJ - length) * ((i - j) / normIJ);
		//	G[neighbor[e][0]] += fNode;
		//	G[neighbor[e][1]] -= fNode;
		//}

		//遍历每条边计算弹力
		for (int e = 0; e < neighbor.size(); e++)
		{
			glm::vec3 Xij = x[neighbor[e][0]] - x[neighbor[e][1]];
			G[neighbor[e][0]] += k * (Xij - glm::normalize(Xij) * lengthN[e]);
			G[neighbor[e][1]] -= k * (Xij - glm::normalize(Xij) * lengthN[e]);
		}

		// gravity
		for (int i = 0; i < x.size(); i++)
		{
			if (i == Lock_Index1 || i == Lock_Index2)
			{
				continue;
			}
			G[i] -= glm::vec3(0.0, -9.8, 0.0) * mass;
		}
	}

	void update()
	{
		vector<glm::vec3> lastX(x.size());
		vector<glm::vec3> xHat(x.size());
		vector<glm::vec3> G(x.size());

		//Initial Setup.
		for (int i = 0; i < x.size(); i++)
		{
			v[i] *= damping;
			xHat[i] = x[i] + v[i] * t;
			lastX[i] = xHat[i];
		}

		// update x by gradient
		float omiga = 0;
		float rho = 0.995f;
		float error = 0.01f;
		vector<glm::vec3> LastX(x.size());
		for (int n = 0; n < 12; n++)
		{
			getGradient(x, xHat, t, G);
			if (n == 0)
				omiga = 1;
			else if (n == 1)
				omiga = 2 / (2 - rho * rho);
			else
				omiga = 4 / (4 - rho * rho * omiga);

			for (int i = 0; i < x.size(); i++)
			{

				if (i == Lock_Index1 || i == Lock_Index2)
				{
					continue;
				}

				if (glm::length(G[i]) < error) continue;

				// Jacobi

				//cout <<"1"<< x[i].x << " " << x[i].y << " " << endl;

				x[i] -= G[i] / (mass / (t * t) + 4 * k);

				//cout << x[i].x << " " << x[i].y << " " << endl;

				//切比雪夫加速
				/*Vector3 oldx = X[i];
				X[i] -= G[i] / (mass / (t * t) + 4 * spring_k);
				X[i] = omiga * X[i] + (1 - omiga) * LastX[i];
				LastX[i] = oldx;*/
			}
		}

		for (int i = 0; i < x.size(); i++)
			v[i] += (x[i] - lastX[i]) / t; //隐式欧拉思想的体现
	}
	
	/*void updateForce()
	{
		for (int e = 0; e < neighbor.size(); e++)
		{
			glm::vec3 i = x[neighbor[e][0]];
			glm::vec3 j = x[neighbor[e][1]];

			float normIJ = glm::sqrt((i - j).x * (i - j).x + (i - j).y * (i - j).y + (i - j).z * (i - j).z);

			glm::vec3 fNode = -k * (normIJ - length) * ((i - j) / normIJ);
			f[neighbor[e][0]] += fNode;
			f[neighbor[e][1]] -= fNode;

			glm::mat3 HeN = k * vectorDotfloatMS(i - j, i - j) / normIJ + k * (1 - length / normIJ) * (glm::mat3(1.0f) - vectorDotfloat(i - j, i - j) / (normIJ * normIJ));
			He.push_back(HeN);
		}
	}*/


	void updateForce()
	{
		for (int e = 0; e < neighbor.size(); e++)
		{
			//i:0 j:1

			glm::vec3 Xij = x[neighbor[e][1]] - x[neighbor[e][0]];
			f[neighbor[e][0]] += k * glm::normalize(Xij) * (glm::sqrt(Xij * Xij) - lengthN[e]);
			f[neighbor[e][1]] -= k * glm::normalize(Xij) * (glm::sqrt(Xij * Xij) - lengthN[e]);

			/*f[neighbor[e][0]] += k * (Xij - glm::normalize(Xij) * length);
			f[neighbor[e][1]] -= k * (Xij - glm::normalize(Xij) * length);*/
		}

		cout << "force" << endl;
		cout << f[1].x << " " << f[1].y << "" << f[1].z << endl;
	}

	void solver()
	{
		// construct 3N*3N Jocobian matrix
		Eigen::SparseMatrix<float> Jocobian(3 * x.size(), 3 * x.size()); 
		vector<Eigen::Triplet<float>> triplets;

		for (int e = 0; e < neighbor.size(); e++)
		{
			/*glm::vec3 i = x[neighbor[e][0]];
			glm::vec3 j = x[neighbor[e][1]];*/

			Eigen::Vector3f i;
			Eigen::Vector3f j;

			i[0] = x[neighbor[e][0]].x;
			i[1] = x[neighbor[e][0]].y;
			i[2] = x[neighbor[e][0]].z;

			j[0] = x[neighbor[e][1]].x;
			j[1] = x[neighbor[e][1]].y;
			j[2] = x[neighbor[e][1]].z;

			/*float l = glm::length(i - j);*/
			float l = (i-j).norm();
			float l0 = lengthN[e];
			//glm::vec3 xij = i - j;
			Eigen::Vector3f xij = j - i;
			Eigen::Matrix3f I;
			I(0, 0) = 1;
			I(0, 1) = 0;
			I(0, 2) = 0;
			I(1, 0) = 0;
			I(1, 1) = 1;
			I(1, 2) = 0;
			I(2, 0) = 0;
			I(2, 1) = 0;
			I(2, 2) = 1;

			Eigen::Matrix3f stiffness = k * (-I + (l0 / l) * (I - xij*xij.transpose()) / (l * l));

			Eigen::Vector3f xijHat = (xij / xij.norm());
			Eigen::Matrix3f stiffness = -k * ((1 - l0 / l) * (I - (xijHat * xijHat.transpose())) + (xijHat * xijHat.transpose()));

			/*cout << stiffness(0, 0) << " " << stiffness(0, 1) << " " << stiffness(0, 2) << endl;
			cout << stiffness(1, 0) << " " << stiffness(1, 1) << " " << stiffness(1, 2) << endl;
			cout << stiffness(2, 0) << " " << stiffness(2, 1) << " " << stiffness(2, 2) << endl;
			cout << "********************" << endl;
			cout << stiffnessN(0, 0) << " " << stiffnessN(0, 1) << " " << stiffnessN(0, 2) << endl;
			cout << stiffnessN(1, 0) << " " << stiffnessN(1, 1) << " " << stiffnessN(1, 2) << endl;
			cout << stiffnessN(2, 0) << " " << stiffnessN(2, 1) << " " << stiffnessN(2, 2) << endl;*/

			//glm::mat3 stiffness = k * (-glm::mat3(1.0f) + (l0 / l) * (glm::mat3(1.0f) - (vectorDotMatMS(xij, xij)) / (l * l)));

			/*cout << stiffness[0][0] << " " << stiffness[0][1] << " " << stiffness[0][2] << endl;
			cout << stiffness[1][0] << " " << stiffness[1][1] << " " << stiffness[1][2] << endl;
			cout << stiffness[2][0] << " " << stiffness[2][1] << " " << stiffness[2][2] << endl;
			cout << endl;*/

			/*triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][1], stiffness[0][0]);
			triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][1] + 1, stiffness[0][1]);
			triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][1] + 2, stiffness[0][2]);
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][1], stiffness[1][0]);
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][1] + 1, stiffness[1][1]);
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][1] + 2, stiffness[1][2]);
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][1], stiffness[2][0]);
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][1] + 1, stiffness[2][1]);
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][1] + 2, stiffness[2][2]);

			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][0], -stiffness[0][0]);
			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][0]+1, -stiffness[0][1]);
			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][0]+2, -stiffness[0][2]);
			triplets.emplace_back(3 * neighbor[e][1]+1, 3 * neighbor[e][0], -stiffness[1][0]);
			triplets.emplace_back(3 * neighbor[e][1]+1, 3 * neighbor[e][0]+1, -stiffness[1][1]);
			triplets.emplace_back(3 * neighbor[e][1]+1, 3 * neighbor[e][0]+2, -stiffness[1][2]);
			triplets.emplace_back(3 * neighbor[e][1]+2, 3 * neighbor[e][0], -stiffness[2][0]);
			triplets.emplace_back(3 * neighbor[e][1]+2, 3 * neighbor[e][0]+1, -stiffness[2][1]);
			triplets.emplace_back(3 * neighbor[e][1]+2, 3 * neighbor[e][0]+2, -stiffness[2][2]);*/

			/*Matrix3f I;
			I << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;

			float dist = (i - j).norm();
			Matrix3f stiffness;
			stiffness = k * (I - lengthN[e] * ((I / dist) - ((i - j) * (i - j).transpose() / pow(dist, 3))));*/

			triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][1], stiffness(0,0));
			triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][1] + 1, stiffness(0, 1));
			triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][1] + 2, stiffness(0, 2));
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][1], stiffness(1, 0));
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][1] + 1, stiffness(1,1));
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][1] + 2, stiffness(1,2));
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][1], stiffness(2,0));
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][1] + 1, stiffness(2, 1));
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][1] + 2, stiffness(2, 2));

			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][0], stiffness(0, 0));
			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][0] + 1, stiffness(0, 1));
			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][0] + 2, stiffness(0, 2));
			triplets.emplace_back(3 * neighbor[e][1] + 1, 3 * neighbor[e][0], stiffness(1, 0));
			triplets.emplace_back(3 * neighbor[e][1] + 1, 3 * neighbor[e][0] + 1, stiffness(1, 1));
			triplets.emplace_back(3 * neighbor[e][1] + 1, 3 * neighbor[e][0] + 2, stiffness(1, 2));
			triplets.emplace_back(3 * neighbor[e][1] + 2, 3 * neighbor[e][0], stiffness(2, 0));
			triplets.emplace_back(3 * neighbor[e][1] + 2, 3 * neighbor[e][0] + 1, stiffness(2, 1));
			triplets.emplace_back(3 * neighbor[e][1] + 2, 3 * neighbor[e][0] + 2, stiffness(2, 2));

			triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][0], -stiffness(0, 0));
			triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][0] + 1, -stiffness(0, 1));
			triplets.emplace_back(3 * neighbor[e][0], 3 * neighbor[e][0] + 2, -stiffness(0, 2));
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][0], -stiffness(1, 0));
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][0] + 1, -stiffness(1, 1));
			triplets.emplace_back(3 * neighbor[e][0] + 1, 3 * neighbor[e][0] + 2, -stiffness(1, 2));
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][0], -stiffness(2, 0));
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][0] + 1, -stiffness(2, 1));
			triplets.emplace_back(3 * neighbor[e][0] + 2, 3 * neighbor[e][0] + 2, -stiffness(2, 2));

			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][1], -stiffness(0, 0));
			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][1] + 1, -stiffness(0, 1));
			triplets.emplace_back(3 * neighbor[e][1], 3 * neighbor[e][1] + 2, -stiffness(0, 2));
			triplets.emplace_back(3 * neighbor[e][1] + 1, 3 * neighbor[e][1], -stiffness(1, 0));
			triplets.emplace_back(3 * neighbor[e][1] + 1, 3 * neighbor[e][1] + 1, -stiffness(1, 1));
			triplets.emplace_back(3 * neighbor[e][1] + 1, 3 * neighbor[e][1] + 2, -stiffness(1, 2));
			triplets.emplace_back(3 * neighbor[e][1] + 2, 3 * neighbor[e][1], -stiffness(2, 0));
			triplets.emplace_back(3 * neighbor[e][1] + 2, 3 * neighbor[e][1] + 1, -stiffness(2, 1));
			triplets.emplace_back(3 * neighbor[e][1] + 2, 3 * neighbor[e][1] + 2, -stiffness(2, 2));
		}

		Jocobian.setFromTriplets(triplets.begin(), triplets.end());

		Jocobian = Jocobian * t * t;

		// construct 3N*3N Mass matrix
		Eigen::SparseMatrix<float> M(3 * x.size(), 3 * x.size());
		vector<Eigen::Triplet<float>> tripletsM;

		for (int i = 0; i < x.size(); i++)
		{
			tripletsM.emplace_back(3 * i, 3 * i, mass);
			tripletsM.emplace_back(3 * i+1, 3 * i+1, mass);
			tripletsM.emplace_back(3 * i+2, 3 * i+2, mass);
		}
		M.setFromTriplets(tripletsM.begin(), tripletsM.end());

		// construct left side
		Eigen::SparseMatrix<float> A(3 * x.size(), 3 * x.size());
		A = M - Jocobian;
		//A = M;

		// construct right side
		Eigen::VectorXf vE(v.size() * 3);
		for (int i = 0; i < v.size(); i++)
		{
			vE[3 * i] = v[i].x;
			vE[3 * i + 1] = v[i].y;
			vE[3 * i + 2] = v[i].z;
		}

		Eigen::VectorXf fE(f.size() * 3);
		for (int i = 0; i < f.size(); i++)
		{
			fE[3 * i] = f[i].x;
			fE[3 * i+1] = f[i].y;
			fE[3 * i+2] = f[i].z;
		}

		Eigen::VectorXf b(3 * x.size());
		b = M * vE - t * fE;

		// solve
		SimplicialLLT<SparseMatrix<float>> solver;
		solver.compute(A);
		Eigen::VectorXf vEN(x.size() * 3);
		vEN = solver.solve(b);

		cout << 3 << endl;

		// update v
		for (int i = 0; i < v.size(); i++)
		{
			v[i].x = vEN[3 * i];
			v[i].y = vEN[3 * i+1];
			v[i].z = vEN[3 * i+2];
		}

		// update x
		for (int i = 0; i < x.size(); i++)
		{
			if (i > 0)
			{
				x[i].x += vEN[3 * i] * t;
				x[i].y += vEN[3 * i + 1] * t;
				x[i].z += vEN[3 * i + 2] * t;
			}
		}

		cout << x[1].x << " " << x[1].y << " " << x[1].z << endl;
	}
};
