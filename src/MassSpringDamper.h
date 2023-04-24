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
#include "Eigen/SparseLU"

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
	Eigen::SparseMatrix<float> M;
	float length = 100.0f;

	// force
	vector<glm::vec3> f;
	//vector<glm::mat3>& He;
	float k = 8000;
	float t = 0.1;
	float mass = 0.1;
	float damping = 20.99;
	float indexConstict = 5;

	// lock
	int Lock_Index1 = 0;
	int Lock_Index2 = 5;

	// original number
	int oNum = 0;
	int flagChangeForce = 1;
	int deleteEdges = 10;

	map<int, int> newADD;

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

	void getMassMatrix(int size)
	{
		M.resize(3 * size, 3 * size);
		M.setZero();

		for (int i = 0; i < x.size(); i++)
		{
			M.coeffRef(3 * i, 3 * i) = mass;
			M.coeffRef(3 * i+1, 3 * i+1) = mass;
			M.coeffRef(3 * i+2, 3 * i+2) = mass;
		}
	}

	void getVelocityandForceList(int num)
	{
		for (int i = 0; i < num; i++)
		{
			this->v.push_back(glm::vec3(0.0));
			if (i > indexConstict)
				this->f.push_back(glm::vec3(0.0, -5.8f, 0.0f));
			else
				this->f.push_back(glm::vec3(0.0f));
		}
	}

	MassSpring()
	{
		this->k = 1;
		this->t = 0.01;
		this->mass = 1;
		this->damping = 20.0;
		this->length = 1.0f;
	}

	void MassSpringInitiate(int Lock_Index1, int Lock_Index2, vector<glm::vec3> vertices, map<tuple<int, int>, int> neighboringEdges)
	{
		this->Lock_Index1 = Lock_Index1;
		this->Lock_Index2 = Lock_Index2;
		this->x = vertices;
		getNeighborList(neighboringEdges);
		getVelocityandForceList(vertices.size());
		getMassMatrix(vertices.size());
		this->k = 180000;
		this->t = 0.01;
		this->mass = 1;
		this->damping = 200.99f;
		this->length = 1.0f;
		this->oNum = vertices.size();
	}

	void updateForce(int extraFnum, glm::vec3 extraForce)
	{
		for (int i = 0; i < x.size(); i++)
		{
			if (i > indexConstict)
				this->f[i] = glm::vec3(0.0, -9.8f, 0.0f) + extraForce;
			else
				this->f[i] = glm::vec3(0.0f);

			/*if (i == extraFnum)
				this->f[i] += extraForce;*/
		}

		for (int e = 0; e < neighbor.size(); e++)
		{
			//i:0 j:1
			glm::vec3 Xij = x[neighbor[e][0]] - x[neighbor[e][1]];
			f[neighbor[e][0]] += -k * (glm::length(Xij) - lengthN[e]) * glm::normalize(Xij);
			f[neighbor[e][1]] -= -k * (glm::length(Xij) - lengthN[e]) * glm::normalize(Xij);

			// damping
			glm::vec3 Vij = v[neighbor[e][0]] - v[neighbor[e][1]];
			f[neighbor[e][0]] += -damping * glm::normalize(Xij) * (Vij * glm::normalize(Xij));
			f[neighbor[e][1]] -= -damping * glm::normalize(Xij) * (Vij * glm::normalize(Xij));
		}

		cout << "force" << endl;
		cout << f[200].x << " " << f[200].y << " " << f[200].z << endl;
		/*glm::vec3 Xij = x[neighbor[200][0]] - x[neighbor[200][1]];
		cout << glm::length(Xij) << endl;*/
	}

	vector<Eigen::SparseMatrix<float>> getJacobian()
	{
		vector<Eigen::SparseMatrix<float>> results;
		// construct 3N*3N Jocobian matrix
		Eigen::SparseMatrix<float> JacobianX(3 * x.size(), 3 * x.size());
		vector<Eigen::Triplet<float>> triplets;

		Eigen::SparseMatrix<float> JacobianV(3 * x.size(), 3 * x.size());
		vector<Eigen::Triplet<float>> tripletsV;

		JacobianX.setZero();
		JacobianV.setZero();
		for (int e = 0; e < neighbor.size(); e++)
		{
			Eigen::Vector3f i;
			Eigen::Vector3f j;

			i[0] = x[neighbor[e][0]].x;
			i[1] = x[neighbor[e][0]].y;
			i[2] = x[neighbor[e][0]].z;

			j[0] = x[neighbor[e][1]].x;
			j[1] = x[neighbor[e][1]].y;
			j[2] = x[neighbor[e][1]].z;

			Eigen::Vector3f Vi;
			Eigen::Vector3f Vj;

			Vi[0] = v[neighbor[e][0]].x;
			Vi[1] = v[neighbor[e][0]].y;
			Vi[2] = v[neighbor[e][0]].z;

			Vj[0] = v[neighbor[e][1]].x;
			Vj[1] = v[neighbor[e][1]].y;
			Vj[2] = v[neighbor[e][1]].z;

			float l = (i - j).norm();
			float l0 = lengthN[e];
			Eigen::Vector3f xij = i - j;
			Eigen::Vector3f vij = Vi - Vj;
			Matrix3f I;
			I << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;

			Eigen::Vector3f xijHat = (xij / xij.norm());
			Eigen::Vector3f vijHat = (vij / vij.norm());
			Eigen::Matrix3f xijMat = xijHat * xijHat.transpose();

			Eigen::Matrix3f stiffnessX1 = -k * ((1-l0/l) * (I-xijMat) + xijMat);
			Eigen::Matrix3f stiffnessX3 = -k * (I - (l0 / l) * (I - xijMat));
			Eigen::Matrix3f stiffnessX2 = -damping * ((xijHat.transpose() * vijHat * I + xijHat * vijHat.transpose()) * ((xijHat * xijHat.transpose() - I) / xij.norm()));
			Eigen::Matrix3f stiffness = stiffnessX3;
			Eigen::Matrix3f stiffnessV = -damping * xijMat;
			// ------------------------------------------------------------------------------------------
			// STIFFNESS X
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					triplets.push_back(Eigen::Triplet<float>(3 * neighbor[e][0] + i, 3 * neighbor[e][1] + j, -stiffness(i, j)));
					triplets.push_back(Eigen::Triplet<float>(3 * neighbor[e][1] + i, 3 * neighbor[e][0] + j, -stiffness(i, j)));
					triplets.push_back(Eigen::Triplet<float>(3 * neighbor[e][0] + i, 3 * neighbor[e][0] + j, stiffness(i, j)));
					triplets.push_back(Eigen::Triplet<float>(3 * neighbor[e][1] + i, 3 * neighbor[e][1] + j, stiffness(i, j)));
				}
			}

			// ------------------------------------------------------------------------------------------
			// STIFFNESS V
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					tripletsV.push_back(Eigen::Triplet<float>(3 * neighbor[e][0] + i, 3 * neighbor[e][1] + j, -stiffnessV(i, j)));
					tripletsV.push_back(Eigen::Triplet<float>(3 * neighbor[e][1] + i, 3 * neighbor[e][0] + j, -stiffnessV(i, j)));
					tripletsV.push_back(Eigen::Triplet<float>(3 * neighbor[e][0] + i, 3 * neighbor[e][0] + j, stiffnessV(i, j)));
					tripletsV.push_back(Eigen::Triplet<float>(3 * neighbor[e][1] + i, 3 * neighbor[e][1] + j, stiffnessV(i, j)));
				}
			}
		}
		JacobianX.setFromTriplets(triplets.begin(), triplets.end());
		JacobianV.setFromTriplets(tripletsV.begin(), tripletsV.end());

		results.push_back(JacobianX);
		results.push_back(JacobianV);

		return results;
	}

	void solver()
	{
		Eigen::VectorXf vE(v.size() * 3);
		for (int i = 0; i < v.size(); i++)
		{
			vE[3 * i] = v[i].x;
			vE[3 * i + 1] = v[i].y;
			vE[3 * i + 2] = v[i].z;
		}

		// Newtons Method
		float NEWTONMAXITERS = 1;
		float error = 0.99f;

		for (int n = 0; n < NEWTONMAXITERS; n++)
		{
			vector<Eigen::SparseMatrix<float>> resultsJacobian = getJacobian();
			Eigen::SparseMatrix<float> stiffnessX = resultsJacobian[0];
			Eigen::SparseMatrix<float> stiffnessV = resultsJacobian[1];

			// right side
			Eigen::SparseMatrix<float> A;
			A = M - t * stiffnessV - t * t * stiffnessX;
			//A = M - t * t * stiffnessX;

			// left side
			Eigen::VectorXf fE(f.size() * 3);
			for (int i = 0; i < f.size(); i++)
			{
				fE[3 * i] = f[i].x;
				fE[3 * i + 1] = f[i].y;
				fE[3 * i + 2] = f[i].z;
			}

			Eigen::VectorXf b(3 * x.size());
			b = t * (fE + t * stiffnessX * vE);
			// b = M * vE + t * fE;c
			// b = M * vE - t * stiffnessV * vE + t * fE;

			// solve
			SparseLU<SparseMatrix<float>> solver;
			solver.compute(A);
			Eigen::VectorXf vEN(x.size() * 3);
			vEN = solver.solve(b);

			vE = vEN;
			//cout << vEN.norm();
			if (vEN.norm() < error)
				break;
		}

		// update v.x
		for (int i = 0; i < oNum; i++)
		{
			if (i > indexConstict)
			{
					v[i].x += vE[3 * i];
					v[i].y += vE[3 * i + 1];
					v[i].z += vE[3 * i + 2];

					x[i].x += v[i].x * t;
					x[i].y += v[i].y * t;
					x[i].z += v[i].z * t;
			}
		}

		// update x
		/*for (int i = 0; i < x.size(); i++)
		{
			if (i > indexConstict)
			{
				x[i].x += v[i].x * t;
				x[i].y += v[i].y * t;
				x[i].z += v[i].z * t;
				if (x[i].y < -5)
				{
					v[i].y = 0;
				}
			}
		}*/
		//cout << "position" << endl;
		// cout << x[101].x << " " << x[101].y << " " << x[101].z << endl;
	}

	void deleteExtraEdges()
	{
		int extraNum = x.size() - oNum;
		if (extraNum != 0)
		{
			cout << extraNum << endl;
			for (int i = 0; i < extraNum; i++)
			{
				x.pop_back();
				v.pop_back();
				f.pop_back();
				neighbor.pop_back();
				lengthN.pop_back();
			}
			getMassMatrix(x.size());
			newADD.clear();
		}
	}

	void groundDetection()
	{
		glm::vec3 p = { 0,-5,0 };
		glm::vec3 n = { 0,1,0 };

		for (int i = 0; i < oNum; i++)
		{
			if (i > indexConstict)
			{
				// check whether collision happens
				if (vectorDotfloat((x[i] - p), n) >= 0)
					continue;

				glm::vec3 vi = v[i];
				if (vectorDotfloat(vi, n) >= 0)
				{
					continue;
				}

				/*glm::vec3 dir = glm::reflect(v[i], n);
				dir = glm::normalize(dir);
				v[i] = glm::length(v[i]) * dir;
				v[i] = glm::vec3(5 * v[i].x, 0.5 * v[i].y, 0.5 * v[i].z);*/
				/*updateForce(i, glm::vec3(0, 1.0, 0));
				flagChangeForce = 1;*/

				if (newADD.find(i) == newADD.end())
				{
					x.push_back(glm::vec3(x[i].x, -5, x[i].z));
					v.push_back(glm::vec3(0.0));
					f.push_back(glm::vec3(0.0));
					getMassMatrix(x.size());
					neighbor.push_back(glm::vec2(i, x.size() - 1));
					lengthN.push_back(glm::length(x[i]-glm::vec3(x[i].x,-5,x[i].z))/2);
					newADD[i] = 0;
				}
			}
		}
	}
};
