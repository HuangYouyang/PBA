#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/trigonometric.hpp>
#include <vector>
#include <iostream>

#include "Sphere.h"

class Particle
{
private:
	glm::vec3 position = glm::vec3(0.0f, 0.5f, 0.0f);
	glm::vec3 velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 color;
	glm::vec3 acceleration = glm::vec3(0.0f, 0.0f, 0.0f);
	
	float radius = 0.1f;
	float edge = 3.0f;
	float rotation;
	float scale;
	float lifeLength; //How long the particle will live for. 
	float age = 0;
	float timeStep = 0.01;
	float alpha = 10.0;

public:
	Particle() :position(glm::vec3(0.0f, 0.5f, 0.0f)), velocity(glm::vec3(0.0f, 0.0f, 0.0f)), color(0), age(0), lifeLength(0), rotation(0) {}

	void renewParticle()
	{
		position = glm::vec3(0.0f, 0.5f, 0.0f);
		velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		acceleration = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	void giveForce(glm::vec3 force)
	{
		acceleration = force;
	}

	glm::vec3 update()
	{
		float con = 0.5f;

		position += timeStep * velocity + con * timeStep * timeStep * acceleration;

		velocity += acceleration * timeStep;

		collisionHandling();

		return position;
	}

	glm::vec3 updateVelocityField() 
	{
		/*
		// update velocity by formula of the velocity field
		velocity = glm::vec3(position.x * 0.005 + 0.005, position.y * 0.005 + position.x, 0);

		// update position
		position = position + velocity * timeStep;

		// collisionHandling();*/

		// circle velocity field
		glm::mat3 model = glm::mat3(glm::cos(glm::radians(90.0f)), -glm::sin(glm::radians(90.0f)), 0, glm::sin(glm::radians(90.0f)), glm::cos(glm::radians(90.0f)), 0, 0, 0, 1);
		glm::vec3 v = alpha * glm::normalize(model * position);

		glm::mat3 I = glm::mat3(1.0f);

		position = glm::inverse(I - alpha * timeStep * model) * position;

		return position;
	}

	
	glm::vec3 updateConcenForceFieldImplicit()
	{
		glm::mat3 model = glm::mat3(glm::cos(glm::radians(90.0f)), -glm::sin(glm::radians(90.0f)), 0, glm::sin(glm::radians(90.0f)), glm::cos(glm::radians(90.0f)), 0, 0, 0, 1);

		glm::mat3 I = glm::mat3(1.0f);

		glm::vec3 positionNext = glm::inverse(I - timeStep * timeStep * alpha * model) * (position + timeStep * velocity);

		velocity += timeStep * alpha * model * positionNext;

		position = positionNext;

		return position;
	}

	glm::vec3 updateConcenForceField()
	{
		// a_n
		glm::mat3 model = glm::mat3(glm::cos(glm::radians(90.0f)), -glm::sin(glm::radians(90.0f)), 0, glm::sin(glm::radians(90.0f)), glm::cos(glm::radians(90.0f)), 0, 0, 0, 1);
		glm::vec3 a = alpha * glm::normalize(model * position);

		velocity  += a * timeStep;
		position += velocity * timeStep;

		return position;
	}

	// get function
	glm::vec3 getPosition()
	{
		return this->position;
	}

	glm::vec3 getVelocity()
	{
		return this->velocity;
	}

	glm::vec3 getColor()
	{
		return this->color;
	}

	float getRotation()
	{
		return this->rotation;
	}

	float getScale()
	{
		return this->scale;
	}

	float getLifeLength()
	{
		return this->lifeLength;
	}

	float getAge()
	{
		return this->age;
	}

	void collisionHandling()
	{
		// collision Detection
		if (position.x - radius <= -edge || position.x + radius >= edge) // left.right
		{
			velocity.x = -velocity.x;
		}
		if (position.y - radius <= -edge || position.y + radius >= edge) // top.bottom
		{
			velocity.y = -velocity.y;
		}
	}
};
