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

using namespace std;

vector<glm::vec3> getVertices(cy::TriMesh& model)
{
	vector<glm::vec3> verticesList;
	
	int verticesN = model.NV();
	for (int i = 0; i < verticesN; i++)
	{
		glm::vec3 r = glm::vec3(model.V(i).x, model.V(i).y, model.V(i).z);
		verticesList.push_back(r);
	}
	return verticesList;
}

vector<glm::vec3> getNormals(cy::TriMesh& model)
{
	vector<glm::vec3> normalsList;

	for (int i = 0; i < model.NF(); i++)
	{
		glm::vec3 v1 = glm::vec3(model.V((model.F(i).v[0])).x, model.V((model.F(i).v[0])).y, model.V((model.F(i).v[0])).z);
		glm::vec3 v2 = glm::vec3(model.V((model.F(i).v[1])).x, model.V((model.F(i).v[1])).y, model.V((model.F(i).v[1])).z);
		glm::vec3 v3 = glm::vec3(model.V((model.F(i).v[2])).x, model.V((model.F(i).v[2])).y, model.V((model.F(i).v[2])).z);

		glm::vec3 faceNormal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
		normalsList.push_back(faceNormal);
	}

	return normalsList;
}

vector<glm::vec3> getVerticesNormals(cy::TriMesh& model)
{
	int verticesN = model.NV();
	vector<glm::vec3> verticesNormalsList(verticesN);
	vector<glm::vec3> verticesNormalsNumList(verticesN);

	for (int i = 0; i < model.NF(); i++)
	{
		glm::vec3 v1 = glm::vec3(model.V((model.F(i).v[0])).x, model.V((model.F(i).v[0])).y, model.V((model.F(i).v[0])).z);
		glm::vec3 v2 = glm::vec3(model.V((model.F(i).v[1])).x, model.V((model.F(i).v[1])).y, model.V((model.F(i).v[1])).z);
		glm::vec3 v3 = glm::vec3(model.V((model.F(i).v[2])).x, model.V((model.F(i).v[2])).y, model.V((model.F(i).v[2])).z);

		glm::vec3 faceNormal = glm::normalize(glm::cross(v2 - v1, v3 - v1));

		verticesNormalsList[(model.F(i).v[0])] += faceNormal;
		verticesNormalsNumList[(model.F(i).v[0])] += 1;

		verticesNormalsList[(model.F(i).v[1])] += faceNormal;
		verticesNormalsNumList[(model.F(i).v[1])] += 1;

		verticesNormalsList[(model.F(i).v[2])] += faceNormal;
		verticesNormalsNumList[(model.F(i).v[2])] += 1;
	}

	for (int i = 0; i < model.NV(); i++)
	{
		verticesNormalsList[i] = verticesNormalsList[i]/ verticesNormalsNumList[i];
	}

	return verticesNormalsList;
}

glm::mat3 calculateIbody(cy::TriMesh &model)
{
	glm::mat3 Ibody = glm::mat3(glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f));
	float m = 1.0f;
	int verticesN = model.NV();
	for (int i = 0; i < verticesN; i++)
	{
		glm::vec3 r = glm::vec3(model.V(i).x, model.V(i).y, model.V(i).z);
		glm::mat3 r0i = glm::mat3(glm::vec3(r), glm::vec3(0.0f), glm::vec3(0.0f));
		glm::mat3 r0i_T = glm::mat3(glm::vec3(r.x, 0, 0), glm::vec3(r.y, 0, 0), glm::vec3(r.z, 0, 0));
		Ibody += (r0i_T * r0i)[0][0] * glm::mat3(1.0f) - r0i * r0i_T;
	}
	return Ibody;
}

glm::vec3 calculateCenterMassPosition(cy::TriMesh & model)
{
	glm::vec3 cmPosition = { 0,0,0 };
	float M = 0.0f;
	float m = 1.0f;
	int verticesN = model.NV();
	for (int i = 0; i < verticesN; i++)
	{
		glm::vec3 r = glm::vec3(model.V(i).x, model.V(i).y, model.V(i).z);
		cmPosition += r * m;
		M += m;
	}
	return cmPosition/M;
}

float calculateMass(cy::TriMesh& model)
{
	float M = 0.0f;
	float m = 1.0f;
	int verticesN = model.NV();
	for (int i = 0; i < verticesN; i++)
	{
		glm::vec3 r = glm::vec3(model.V(i).x, model.V(i).y, model.V(i).z);
		M += m;
	}
	return M;
}

glm::mat3 star(glm::vec3 &R)
{
	return glm::mat3(glm::vec3(0, R.z, -R.y), glm::vec3(-R.z, 0, R.x), glm::vec3(R.y, -R.x, 0));
}

//glm::quaternionMulti(glm::)

class RigidBody
{
public:
	// model
	cy::TriMesh model;
	vector<glm::vec3> verticesList;
	vector<glm::vec3> normalsList;
	vector<glm::vec3> verticesNormalList;

	// constant quantities
	float mass;
	glm::mat3 Ibody, Ibodyinv;

	// state variables
	glm::vec3 xWorld;
	glm::vec3 x, P, L;
	glm::mat3 R;
	glm::mat4 rotMat;
	glm::quat q;

	// derived variables
	glm::mat3 Iinv;
	glm::vec3 v, w; // linear velocity & angular velocity
	
	// computed quantities
	glm::vec3 F, T; // force and torque

	float flag = 0.0f;

	float timeStep = 0.2f;

	// variables for collision
	float restitution = 0.5f;
	float restitution_T = 0.2f;

	RigidBody() 
	{
		this->mass = 10000;
		this->x = glm::vec3(0, 0, 0);
		this->v = { 0,0,0 };
		this->w = { 0,0,0 };
		this->R = glm::mat3(1.0f);
		this->L = { 0,0,0 };
		this->P = { 0,0,0 };
	}

	RigidBody(cy::TriMesh model)
	{
		this->model = model;
		this->verticesList = getVertices(this->model);
		this->normalsList = getNormals(this->model);
		this->verticesNormalList = getVerticesNormals(this->model);
		this->mass = calculateMass(this->model);
		this->x = calculateCenterMassPosition(this->model);
		this->xWorld = this->x;
		this->Ibody = calculateIbody(this->model);
		this->Ibodyinv = glm::inverse(Ibody);
		this->v = { 0,0,0 };
		this->w = { 0,0,0 };
		this->R = glm::mat3(1.0f);
		this->L = { 0,0,0 };
		this->P = { 0,0,0 };
		this->q = glm::quat(1, 0, 0, 0);

		ComputeForceAndTorque(this);
	}

	// Applies a force at a point in the body, inducing some torque.
	void ComputeForceAndTorque(RigidBody* rigidBody) {
		glm::vec3 f = { 0, -9.8 , 0 };
		rigidBody->F = f;
		// r is the 'arm vector' that goes from the center of mass to the point of force application
		glm::vec3 r = { this->x.x, this->x.y, this->x.z};
		// glm::vec3 r = { 0.696505, 0.209344, -0.059058 };
		// glm::vec3 r = { 0.696505, this->xWorld.y, this->xWorld.z };
		rigidBody->T = glm::cross((mat3_cast(this->q))*(r-this->xWorld),f);
	}

	void update2() {
		// update position
		glm::vec3 a = this->F / mass;
		this->v = v + a * timeStep;
		this->x = x + v * timeStep;

		//this->P = this->P + this->F * timeStep;
		//this->v = this->P / this->mass;
		//this->x = x + v * timeStep; 

		this->L = this->L + this->T * timeStep;
		this->R = mat3_cast(this->q);
		this->Iinv = this->R * glm::inverse(this->Ibody) * glm::transpose(this->R);

		this->w = this->w + this->Iinv * timeStep * this->T;
		glm::vec3 dw = 0.5f * w * timeStep;
		glm::quat qw = glm::quat(0, dw.x, dw.y, dw.z);

		this->q = glm::normalize(this->q + glm::cross(qw, this->q));

		//std::cout << q.x << q.y << q.z << std::endl;

		this->rotMat = glm::mat4_cast(this->q);

		ComputeForceAndTorque(this);
	}

	void contactDetection(RigidBody* r);

	void contactDetection2(glm::vec3 P, glm::vec3 n);

	void contactDetection3(glm::vec3 p, glm::vec3 n, vector<RigidBody> rigidBodyList);
};

float vectorDotfloat(glm::vec3 a, glm::vec3 b)
{
	glm::mat3 r0i = glm::mat3(a, glm::vec3(0.0f), glm::vec3(0.0f));
	glm::mat3 r0i_T = glm::mat3(glm::vec3(b.x, 0, 0), glm::vec3(b.y, 0, 0), glm::vec3(b.z, 0, 0));
	return (r0i_T * r0i)[0][0];
}

class Contact
{
public:
	RigidBody* a; // body containing vertex
	RigidBody* b; // body containing face

	glm::vec3 p, // world-space vertex location
		      n; // outwards pointing normal of the face

	bool vf; // true if vertex and face contact

	Contact() {
		p = glm::vec3(1.0f);
	};

	// return point velocity
	glm::vec3 getPointVelocity(RigidBody* r, glm::vec3 point)
	{
		return r->v + glm::cross(r->w, (r->R)*(point - r->xWorld)); // v = v + w x (R*ri)
	}

	// return true if body are in colliding contact
	bool colliding(Contact* c, glm::vec3 point)
	{
		float threshold = 0.1f;

		glm::vec3 aPointVelocity = getPointVelocity(c->a, point);
		glm::vec3 bPointVelocity = getPointVelocity(c->b, point);

		glm::mat3 r0i = glm::mat3(glm::vec3(aPointVelocity - bPointVelocity), glm::vec3(0.0f), glm::vec3(0.0f));
		glm::mat3 r0i_T = glm::mat3(glm::vec3(c->n.x, 0, 0), glm::vec3(c->n.y, 0, 0), glm::vec3(c->n.z, 0, 0));
		float vrel = (r0i_T * r0i)[0][0]; // vrel-

		if (vrel > threshold) // moving away
			return false;
		else if (vrel > -threshold) // resting contact
			return false;
		else
			return true;
	}

	void collision(Contact* c, float epsilon)
	{
		glm::vec3 aPointVelocity = getPointVelocity(c->a, c->p);
		glm::vec3 bPointVelocity = getPointVelocity(c->b, c->p);
		glm::vec3 n = c->n;
		glm::vec3 rA = c->a->R * (c->p - c->a->xWorld);
		glm::vec3 rB = c->b->R * (c->p - c->b->xWorld);

		float vrel = vectorDotfloat(n, (aPointVelocity - bPointVelocity));
		float numerator = -(1 + epsilon) * vrel;

		float term1 = 1 / c->a->mass;
		float term2 = 1 / c->b->mass;
		float term3 = vectorDotfloat(n, glm::cross(c->a->Iinv * glm::cross(rA, n), rA));
		float term4 = vectorDotfloat(n, glm::cross(c->b->Iinv * glm::cross(rB, n), rB));

		// compute the impulse magnitude
		float j = numerator / (term1 + term2 + term3 + term4);
		glm::vec3 force = j * n;
		
		// std::cout << force.x << "  " << force.y << "  " << force.z << std::endl;

		// apply the impulse to the bodies
		c->a->P += force;
		c->b->P -= force;
		c->a->L += glm::cross(rA, force);
		c->b->L += glm::cross(rB, force);

		// recompute velocity
		c->a->v = c->a->P / c->a->mass;
		c->b->v = c->b->P / c->b->mass;

		std::cout << c->a->v.x << "  " << c->a->v.y << "  " << c->a->v.z << std::endl;

		c->a->w = c->a->Iinv * c->a->L;
		c->b->w = c->b->Iinv * c->b->L;
	}

};

// return point velocity
glm::vec3 getPointVelocity(RigidBody* r, glm::vec3 point)
{
	return r->v + glm::cross(r->w, (r->R) * (point - r->xWorld)); // v = v + w x (R*ri)
}

void RigidBody::contactDetection(RigidBody* r)
{
	int verticesN = r->model.NV();
	for (int i = 0; i < verticesN; i++)
	{
		glm::vec3 p = glm::vec3(r->model.V(i).x, r->model.V(i).y, r->model.V(i).z);

		glm::vec3 pMoved = r->R * (r->xWorld - p) + r->x;

		if (pMoved.y < -4)
		{
			Contact c = Contact();

			std::cout << "*******************************" << std::endl;

			c.p = p; // collision point
			c.a = r; // rigidbody A

			// create plane
			RigidBody b = RigidBody();
			c.b = &b;
			c.b->mass = 100000000;
			c.n = glm::vec3{ 0, 1, 0 };

			c.collision(&c, 0.01);
			break;
		}
	}
}


void RigidBody::contactDetection2(glm::vec3 p, glm::vec3 n)
{
	int verticesN = this->model.NV();

	glm::vec3 avgCollisionP = glm::vec3(0, 0, 0);
	float numCollision = 0;

	for (int i = 0; i < verticesN; i++)
	{
		glm::vec3 ri = glm::vec3(this->model.V(i).x, this->model.V(i).y, this->model.V(i).z);

		glm::vec3 xi = this->R * ri + this->x; // r in the world space position

		// check whether collision happens
		if (vectorDotfloat((xi - p), n) >= 0)
			continue;

		glm::vec3 vi = v + glm::cross(w, R * ri);
		if (vectorDotfloat(vi, n) >= 0)
		{
			continue;
		}

		avgCollisionP += ri;
		numCollision++;
	}


	// No collision
	if (numCollision == 0)
		return;

	// Collision
	glm::vec3 ri = avgCollisionP / numCollision;
	glm::vec3 Rri = R * ri;
	glm::vec3 vi = v + glm::cross(w, Rri);

	// Vi_new
	glm::vec3 vi_N = vectorDotfloat(vi, n) * n;
	glm::vec3 vi_T = vi - vi_N;
	float a = glm::max(1.0f - restitution_T * (1.0f + restitution) * glm::length(vi_N) / glm::length(vi_T), 0.0f);

	glm::vec3 vi_new_N = -restitution * vi_N;
	glm::vec3 vi_new_T = a * vi_T;
	glm::vec3 vi_new = vi_new_N + vi_new_T;

	// J
	glm::mat3 I = R * Ibody * glm::transpose(R);
	glm::mat3 K = (1 / mass) * glm::mat3(1.0f) - star(Rri) * glm::inverse(I) * star(Rri);
	glm::vec3 J = glm::inverse(K)* (vi_new - vi);

	// update v.w
	v += 1.0f / mass * J;
	w += glm::inverse(I) * (glm::cross(Rri, J));
}

void RigidBody::contactDetection3(glm::vec3 p, glm::vec3 n, vector<RigidBody> rigidBodyList)
{
	glm::vec3 Jsum = glm::vec3(0.0f);
	glm::vec3 Rrisum = glm::vec3(0.0f);
	float Collisionsum = 0;

	// check all the rigidbodies
	for (int numRigidBody = 0; numRigidBody < rigidBodyList.size(); numRigidBody++)
	{
		float normalsN = rigidBodyList[numRigidBody].normalsList.size();
		for (int j = 0; j < normalsN; j++)
		{
			n = rigidBodyList[numRigidBody].normalsList[j];
			int verticesN = this->model.NV();

			glm::vec3 avgCollisionP = glm::vec3(0, 0, 0);
			float numCollision = 0;

			for (int i = 0; i < verticesN; i++)
			{
				glm::vec3 ri = glm::vec3(this->model.V(i).x, this->model.V(i).y, this->model.V(i).z);

				glm::vec3 xi = this->R * ri + this->x; // r in the world space position

				// check whether collision happens
				if (vectorDotfloat((xi - p), n) >= 0)
					continue;

				glm::vec3 vi = v + glm::cross(w, R * ri);
				if (vectorDotfloat(vi, n) >= 0)
				{
					continue;
				}

				avgCollisionP += ri;
				numCollision++;
			}

			// No collision
			if (numCollision == 0)
				return;

			// Collision
			glm::vec3 ri = avgCollisionP / numCollision;
			glm::vec3 Rri = R * ri;
			glm::vec3 vi = v + glm::cross(w, Rri);

			// Vi_new
			glm::vec3 vi_N = vectorDotfloat(vi, n) * n;
			glm::vec3 vi_T = vi - vi_N;
			float a = glm::max(1.0f - restitution_T * (1.0f + restitution) * glm::length(vi_N) / glm::length(vi_T), 0.0f);

			glm::vec3 vi_new_N = -restitution * vi_N;
			glm::vec3 vi_new_T = a * vi_T;
			glm::vec3 vi_new = vi_new_N + vi_new_T;

			// J
			glm::mat3 I = R * Ibody * glm::transpose(R);
			glm::mat3 K = (1 / mass) * glm::mat3(1.0f) - star(Rri) * glm::inverse(I) * star(Rri);
			glm::vec3 J = glm::inverse(K) * (vi_new - vi);

			Jsum += J;
			Collisionsum += 1;
			Rrisum += Rri;
		}
	}
	
	// update v.w
	v += 1.0f / mass * Jsum;
	glm::mat3 I = R * Ibody * glm::transpose(R);
	w += glm::inverse(I) * (glm::cross(Rrisum/Collisionsum, Jsum));
}