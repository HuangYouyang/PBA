#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Arrow
{
private:
    float startX, startY, startZ;
    float endX, endY, endZ;

public:
    Arrow() :startX(0), startY(0), startZ(0), endX(0), endY(0), endZ(0) {}

    void setStart(float startX, float startY, float startZ)
    {
        this->startX = startX;
        this->startY = startY;
        this->startZ = startZ;
    }

    // draw arrow
    void draw(float eX, float eY, float eZ)
    {
        int attribVertex = 0, attribNormal = 1, attribTexCoord = 2;

        GLfloat lineVertices[] = {
            this->startX, this->startY, this->startZ,
            eX / 1000, eY / 1000, eZ / 1000
        };



        cout << eX << "    " << eY << "    " << eZ << "    " << endl;

        glLineWidth(10);

        // vertex data VBO
        GLuint vboId;
        glGenBuffers(1, &vboId);
        glBindBuffer(GL_ARRAY_BUFFER, vboId);
        glBufferData(GL_ARRAY_BUFFER,
            sizeof(lineVertices),
            lineVertices,
            GL_STATIC_DRAW);

        // bind
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 3 * sizeof(GL_FLOAT), (void*)0);
        glEnableVertexAttribArray(0);

        // draw line
        glDrawArrays(GL_LINES, 0, 2);
    }
};