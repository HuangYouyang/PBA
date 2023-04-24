#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <math.h>

#include "shader.h"
#include "camera.h"
#include "mesh.h"
#include "model.h"
#include "utils.h"
#include "light.h"
#include "Sphere.h"
#include "Line.h"
#include "Particle.h"
#include "cyTriMesh.h"
#include "cyCore.h"
#include "cyMatrix.h"
#include "rigidBody.h"


cy::TriMesh dragonMesh, armaMesh;

struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};
Ray getRayFromMouse(float mouseX, float mouseY, int windowWidth, int windowHeight, glm::mat4 viewMatrix, glm::mat4 projectionMatrix);
bool intersectRayPoint(Ray ray, glm::vec3 point, float& t);

#define _USE_MATH_DEFINES
#include <cmath>

unsigned int depthMapFBO, depthMap;
static const int SHADOW_WIDTH = 800, SHADOW_HEIGHT = 600;
bool model_draw = true, display_corner = false, move_light = false;

// mouse control
bool leftButton = false, rightButton = false, rightFlag = false;
float preX, preY;
float positionX = 0, positionY = 0;
float startX, startY, startZ;
float scaleNumber = 1.0f;
float scalePreX = 0, scalePreY = 0, positionPreX = 0;
glm::vec3 lastVertice = glm::vec3{ 0,0,0 };

// change mode
float modeFlag = 1;
bool modeChange = false;

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 20.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// lighting
glm::vec3 LightPositions[] = {
    glm::vec3(1.2f, 1.0f, 2.0f),
    glm::vec3(1.2f, 2.0f, 0.0f),
    glm::vec3(-1.2f, 2.0f, 2.0f),
    glm::vec3(-1.2f, 2.0f, 0.0f)
};
glm::vec3& lightPos(LightPositions[0]);

bool doesVectorPassThroughPoint(glm::vec3 vectorStart, glm::vec3 vectorDirection, glm::vec3 point) {
    glm::vec3 pointToStart = vectorStart - point;
    float distance = glm::length(pointToStart);
    float magnitude = glm::length(vectorDirection);

    return glm::abs(distance - magnitude) < 0.0001f;
}

int main()
{
    Particle particle = Particle();
    Particle particleImplicit = Particle();
    Particle particleVelocityFiled = Particle();
    Particle particleF = Particle();

    // ------------------------------------------------------------------
    // glfw: initialize and configure
    // ------------------------------------------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // ------------------------------------------------------------------
    // glfw window creation
    // ------------------------------------------------------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "glad framework", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // ------------------------------------------------------------------
    // glad: load all OpenGL function pointers
    // ------------------------------------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    glEnable(GL_DEPTH_TEST);

    // ------------------------------------------------------------------
    // build and compile our shader programs
    // ------------------------------------------------------------------

    Shader lightingShader("shaders/shadow/shadow.vert", "shaders/shadow/shadow.frag");

    // load model
    Model dragon("assets/bunny/dragon_8kface.obj");
    Model arma("assets/bunny/armadillo.obj");
    Model arrow("assets/bunny/arrow.obj");
    dragonMesh.LoadFromFileObj("assets/bunny/dragon_8kface.obj");
    armaMesh.LoadFromFileObj("assets/bunny/armadillo.obj");

    std::cout << dragon.meshes[0].indices[0] << std::endl;
    std::cout << dragon.meshes[0].indices[1] << std::endl;
    std::cout << dragon.meshes[0].indices[2] << std::endl;

    RigidBody dragonRB(dragonMesh);
    RigidBody dragonRB2(dragonMesh);
    RigidBody dragonRB3(dragonMesh);
    RigidBody armaRB(armaMesh);

    Light lights(LightPositions, 1);

    // ------------------------------------------------------------------
    // shader configuration
    // ------------------------------------------------------------------
    lightingShader.use();
    lightingShader.setInt("material.diffuse", 0);
    lightingShader.setInt("material.specular", 1);
    lightingShader.setInt("shadowMap", 2);
    lightingShader.setFloat("material.shininess", 64);

    // ------------------------------------------------------------------
    // render loop
    // ------------------------------------------------------------------
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        float currentFrame = glfwGetTime();
        // lightingShader.setFloat("time",currentFrame);
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        processInput(window);

        // render setup
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_STENCIL_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilFunc(GL_ALWAYS, 1, 0XFF);
        glStencilMask(0XFF);
        lightingShader.use();

        // ------------------------------------------------------------------
        // render
        // ------------------------------------------------------------------
        // be sure to activate shader when setting uniforms/drawing objects
        float scale = 1.02;

        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 1.0f, 1000.0f);
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 tmpmodel = glm::scale(model, glm::vec3(scale, scale, scale));
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 lightSpaceTrans = glm::lookAt(lightPos, glm::vec3(0.0f), camera.WorldUp);
        const auto renderScene = [&](Shader& shader)
        {

            int flag = glGetUniformLocation(lightingShader.ID, "flag");
            glUniform1i(flag, 1);

            if (modeFlag == 1)
            {
                // render Dragon1 ****************************************************************************************
                vector<RigidBody> rigidBodyList1 = { dragonRB2, dragonRB3 };

                glm::mat4 modelRotation = glm::mat4(1.0f);
                modelRotation = dragonRB.rotMat * model;
                glm::mat4 modelTranslate = glm::mat4(1.0f);
                modelTranslate = glm::translate(modelTranslate, glm::vec3(dragonRB.x.x, dragonRB.x.y, dragonRB.x.z));
                glm::mat4 model = glm::mat4(1.0f);
                model = modelTranslate * modelRotation * model;

                shader.setMat4("model", model);
                std::cout << dragon.meshes[0].vertices.size() << std::endl;
                dragon.Draw(shader);
                dragonRB.update2();

                // contact detection
                glm::vec3 planeDown = glm::vec3(0.0, -4.0, 0.0);
                dragonRB.contactDetection2(planeDown, { 0,1,0 });
                glm::vec3 planeRight = glm::vec3(4.0, 0.0, 0.0);
                dragonRB.contactDetection2(planeRight, { -1,0,0 });
                //dragonRB.contactDetection3(planeRight, { -1,0,0 }, rigidBodyList1);

                // render Dragon2 *******************************************************************************************
                /*vector<RigidBody> rigidBodyList2 = { dragonRB, dragonRB3 };

                modelRotation = glm::mat4(1.0f);
                modelRotation = dragonRB2.rotMat * model;
                modelTranslate = glm::mat4(1.0f);
                modelTranslate = glm::translate(model, glm::vec3(-1, -2, 0));
                modelTranslate = glm::translate(model, glm::vec3(dragonRB2.x.x, dragonRB2.x.y, dragonRB2.x.z));
                model = glm::mat4(1.0f);
                model = modelTranslate * modelRotation * model;

                shader.setMat4("model", model);
                dragon.Draw(shader);
                dragonRB2.update2();

                planeDown = glm::vec3(0.0, -4.0, 0.0);
                dragonRB2.contactDetection2(planeDown, { 0,1,0 });
                planeRight = glm::vec3(4.0, 0.0, 0.0);
                dragonRB2.contactDetection2(planeRight, { -1,0,0 });
                dragonRB2.contactDetection3(planeRight, { -1,0,0 }, rigidBodyList2);*/

                // render Dragon3 *******************************************************************************************
                /*vector<RigidBody> rigidBodyList3 = { dragonRB2, dragonRB };

                modelRotation = glm::mat4(1.0f);
                modelRotation = dragonRB.rotMat * model;
                modelTranslate = glm::mat4(1.0f);
                modelTranslate = glm::translate(modelTranslate, glm::vec3(1, -2, 0));
                modelTranslate = glm::translate(modelTranslate, glm::vec3(dragonRB3.x.x, dragonRB3.x.y, dragonRB3.x.z));
                model = glm::mat4(1.0f);
                model = modelTranslate * modelRotation * model;

                shader.setMat4("model", model);
                dragon.Draw(shader);
                dragonRB3.update2();

                planeDown = glm::vec3(0.0, -4.0, 0.0);
                dragonRB3.contactDetection2(planeDown, { 0,1,0 });
                planeRight = glm::vec3(4.0, 0.0, 0.0);
                dragonRB3.contactDetection2(planeRight, { -1,0,0 });
                dragonRB3.contactDetection3(planeRight, { -1,0,0 }, rigidBodyList3);*/

                // render Monster
                model = glm::mat4(1.0f);
                model = glm::scale(model, glm::vec3(0.00002f, 0.00002f, 0.00002f));
                model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f));
                shader.setMat4("model", model);
                // arma.Draw(shader);

                // render Plane
                model = glm::mat4(1.0f);
                model = glm::translate(model, glm::vec3(0.0f, -8.4f, 0.0f));
                shader.setMat4("model", model);
                renderPlane();

                model = glm::mat4(1.0f);
                model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
                model = glm::translate(model, glm::vec3(0.0f, -8.4f, 0.0f));
                shader.setMat4("model", model);
                renderPlane();

                // std::cout << dragonRB.x.x << "   " << dragonRB.x.y << "    " << dragonRB.x.z << std::endl;

                // Mouse interaction
                // getting cursor position
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);

                if (leftButton)
                {

                    Ray R = Ray();
                    R = getRayFromMouse(xpos, ypos, SCR_WIDTH, SCR_HEIGHT, view, projection);

                    if ((xpos - positionPreX) == 0)
                    {
                        glm::vec3 ri = dragonRB.R * lastVertice + dragonRB.x;

                        model = glm::mat4(1.0f);
                        model = glm::translate(model, ri);
                        shader.setMat4("model", model);
                        arrow.Draw(shader);
                        
                    }
                    else
                    {
                        int verticesN = dragonRB.model.NV();
                        float zNearest = 1000.0f;
                        glm::vec3 pNearest = glm::vec3(0.0f);
                        glm::vec3 piNearest = glm::vec3(0.0f);

                        for (int i = 0; i < verticesN; i++)
                        {
                            glm::vec3 r = dragonRB.verticesList[i];
                            glm::vec3 ri = dragonRB.R * r + dragonRB.x;

                            if (doesVectorPassThroughPoint(R.origin, R.direction, ri))
                            {
                                if (zNearest > ri.z)
                                {
                                    zNearest = ri.z;
                                    pNearest = r;
                                    piNearest = ri;
                                }
                            }
                        }

                        model = glm::mat4(1.0f);
                        model = glm::translate(model, piNearest);
                        shader.setMat4("model", model);
                        arrow.Draw(shader);
                        lastVertice = pNearest;
                    }
                    positionPreX = xpos;
                    
                }
                else if (rightButton)
                {
                    if (rightFlag == false)
                    {
                        // roate
                        positionX += 0;
                        positionY += 0;
                        rightFlag = true;
                    }
                    else
                    {
                        // roate
                        positionX += (xpos - preX) * 0.1f;
                        positionY += (ypos - preY) * 0.1f;
                    }
                    preX = xpos;
                    preY = ypos;

                    xpos = xpos / 1000;
                    ypos = ypos / 1000;

                    glm::vec3 ri = dragonRB.R * lastVertice + dragonRB.x;

                    model = glm::mat4(1.0f);
                    model = glm::translate(model, ri);
                    model = glm::rotate(model, glm::radians(positionX), glm::vec3(0.0f, 0.0f, 1.0f));
                    model = glm::rotate(model, glm::radians(positionY), glm::vec3(0.0f, 1.0f, 0.0f));
                    shader.setMat4("model", model);
                    arrow.Draw(shader);

                    glm::vec3 forceDirection = glm::vec3(20.0f, 0.0f, 0.0f);
                    forceDirection = glm::vec3(model * glm::vec4(forceDirection, 1.0f));

                    dragonRB.F += forceDirection;
                    dragonRB.update2();
                }
            }
            if (modeFlag == 2)
            {
                if (modeChange == false)
                {
                    modeChange = true;
                    particleF.renewParticle();
                }

                // getting cursor position
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);

                //renderArrow();

                glm::vec3 particlePosition = particleF.update();

                model = glm::mat4(1.0f);
                model = glm::translate(model, particlePosition);
                shader.setMat4("model", model);
                renderSphere();

                if (leftButton)
                {
                    // scale
                    scaleNumber += (xpos - scalePreX) * 0.001f;

                    if (scaleNumber >= 1)
                    {
                        scaleNumber = 1;
                    }
                    else if (scaleNumber <= 0.35)
                    {
                        scaleNumber = 0.35;
                    }

                    scalePreX = xpos;

                    // giveForce
                    model = glm::mat4(1.0f);
                    model = glm::rotate(model, glm::radians(positionX), glm::vec3(0.0f, 0.0f, 1.0f));
                    glm::vec4 d = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
                    glm::vec3 forceDirection = glm::vec3(model * d);

                    glm::vec3 force = scaleNumber * 20.0f * glm::normalize(forceDirection);

                    particleF.giveForce(force);
                }
                else if (rightButton)
                {
                    if (rightFlag == false)
                    {
                        // roate
                        positionX += 0;
                        positionY += 0;
                        rightFlag = true;
                    }
                    else
                    {
                        // roate
                        positionX += (xpos - preX) * 0.1f;
                        positionY += (ypos - preY) * 0.1f;
                    }
                    preX = xpos;
                    preY = ypos;

                    xpos = xpos / 1000;
                    ypos = ypos / 1000;
                }
                if (leftButton || rightButton)
                {
                    glm::mat4 model = glm::mat4(1.0f);
                    model = glm::translate(model, particlePosition);
                    model = glm::rotate(model, glm::radians(positionX), glm::vec3(0.0f, 0.0f, 1.0f));
                    model = glm::scale(model, glm::vec3(scaleNumber, 0.5f, 1.0f));
                    shader.setMat4("model", model);

                    int oc = glGetUniformLocation(lightingShader.ID, "ArrowColor");

                    // colormap 
                    glm::vec3 ArrowColor((1 + 0.35 / 0.65) * scaleNumber + (0.35 / -0.65), 0.0f, (-1 / 0.65) * scaleNumber + (1 / 0.65));

                    glUniform3fv(oc, 1, glm::value_ptr(ArrowColor));

                    int flag = glGetUniformLocation(lightingShader.ID, "flag");
                    glUniform1i(flag, 2);

                    // arrow.Draw(lightingShader);
                }
            }
        };

        int viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        lightingShader.use();
        lightingShader.setVec2("pickPosition", glm::vec2(lastX / viewport[2] * 2 - 1.0f, (1 - lastY / viewport[3]) * 2 - 1.0f));
        lightingShader.setMat4("lightView", glm::perspective(glm::radians(89.0f), (float)SHADOW_WIDTH / SHADOW_HEIGHT, 0.1f, 10.0f) * lightSpaceTrans);
        view = camera.GetViewMatrix();
        lightingShader.setVec3("objectColor", 1.0f, 0.0f, 0.0f);
        lightingShader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
        lightingShader.setVec3("lightPos", lightPos);
        lightingShader.setVec3("viewPos", camera.Position);

        // view/projection transformations
        lightingShader.setMat4("projection", projection);
        lightingShader.setMat4("view", view);

        // world transformation
        lightingShader.setMat4("model", model);

        renderScene(lightingShader);
        // also draw the lamp object
        lights.Draw(camera);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // ------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    if (move_light) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            lightPos += 2.5f * deltaTime * camera.Front;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            lightPos -= 2.5f * deltaTime * camera.Front;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            lightPos -= 2.5f * deltaTime * camera.Right;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            lightPos += 2.5f * deltaTime * camera.Right;
    }
    else {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(BACKWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(RIGHT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    // if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
    //     model_draw=!model_draw;
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
        display_corner = !display_corner;
    if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
        move_light = !move_light;
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    //if (firstmouse)
    //{
    //    lastx = xpos;
    //    lasty = ypos;
    //    firstmouse = false;
    //}

    //float xoffset = xpos - lastx;
    //float yoffset = lasty - ypos; // reversed since y-coordinates go from bottom to top

    //lastx = xpos;
    //lasty = ypos;

    //camera.processmousemovement(xoffset, yoffset);

}

// glfw: whenever the click the mouse button, this callback is called
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS)
        {
            leftButton = true;

            //getting cursor position
            double xpos, ypos;
            //getting cursor position
            glfwGetCursorPos(window, &xpos, &ypos);
        }
        else
            leftButton = false;
    }
    else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS)
        {
            rightButton = true;

            //getting cursor position
            double xpos, ypos;
            //getting cursor position
            glfwGetCursorPos(window, &xpos, &ypos);
        }
        else
        {
            rightFlag = false;
            rightButton = false;
        }
    }
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_1 && action == GLFW_PRESS)
    {
        modeFlag = 1;
        modeChange = false;
    }
    else if (key == GLFW_KEY_2 && action == GLFW_PRESS) // different velocity field
    {
        modeFlag = 2;
        modeChange = false;
    }
}


void renderPlane() {

    static unsigned int planeVBO, planeVAO = 0;
    static float planeVertices[] = {
        // positions            // normals         // texcoords
         25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,  25.0f,  0.0f,
        -25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,   0.0f,  0.0f,
        -25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,   0.0f, 25.0f,

         25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,  25.0f,  0.0f,
        -25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,   0.0f, 25.0f,
         25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,  25.0f, 25.0f
    };
    if (planeVAO == 0) {
        // plane VAO
        glGenVertexArrays(1, &planeVAO);
        glGenBuffers(1, &planeVBO);
        glBindVertexArray(planeVAO);
        glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glBindVertexArray(0);
    }
    glBindVertexArray(planeVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}
void renderCube(int light) {
    static float vertices[] = {
        // positions          // normals           // texture coords
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,

        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,

        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  1.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,

        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f
    };

    // configure the cube's VAO (and VBO)
    static unsigned int VBO = -1, cubeVAO = -1, lightCubeVAO;

    if (cubeVAO == -1) {
        glGenVertexArrays(1, &cubeVAO);
        glGenBuffers(1, &VBO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindVertexArray(cubeVAO);

        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);

        // second, configure the light's VAO (VBO stays the same; the vertices are the same for the light object which is also a 3D cube)
        glGenVertexArrays(1, &lightCubeVAO);
        glBindVertexArray(lightCubeVAO);

        // we only need to bind to the VBO (to link it with glVertexAttribPointer), no need to fill it; the VBO's data already contains all we need (it's already bound, but we do it again for educational purposes)
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }
    glBindVertexArray(light ? lightCubeVAO : cubeVAO);
    //glDrawArrays(GL_TRIANGLES, 0, 36);
}

void renderArrow()
{
    int attribVertex = 0, attribNormal = 1, attribTexCoord = 2;

    GLfloat lineVertices[] = {
        -3, -3, 0,
        -3, 3, 0,
        -3, 3, 0,
        3, 3, 0,
        3, 3, 0,
        3, -3, 0,
        3, -3, 0,
        -3, -3, 0,
    };

    //glLineWidth(1);

    // vertex data VBO
    GLuint vboId;
    glGenBuffers(1, &vboId);
    glBindBuffer(GL_ARRAY_BUFFER, vboId);
    glBufferData(GL_ARRAY_BUFFER,
        sizeof(lineVertices),
        lineVertices,
        GL_STATIC_DRAW);

    // bind
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GL_FLOAT), (void*)0);
    glEnableVertexAttribArray(0);

    // draw line
   /* glDrawElements(GL_TRIANGLES,
        sphere.getIndexCount(),
        GL_UNSIGNED_INT,
        (void*)0);*/
    glDrawArrays(GL_LINES, 0, 8);
}

void renderSphere()
{
    int attribVertex = 0, attribNormal = 1, attribTexCoord = 2;

    Sphere sphere(0.1f, 36, 18);

    // vertex data VBO
    GLuint vboId;
    glGenBuffers(1, &vboId);
    glBindBuffer(GL_ARRAY_BUFFER, vboId);
    glBufferData(GL_ARRAY_BUFFER,
        sphere.getInterleavedVertexSize(),
        sphere.getInterleavedVertices(),
        GL_STATIC_DRAW);

    // index data VBO
    GLuint iboId;
    glGenBuffers(1, &iboId);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, iboId);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        sphere.getIndexSize(),
        sphere.getIndices(),
        GL_STATIC_DRAW);

    // bind
    glBindBuffer(GL_ARRAY_BUFFER, vboId);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, iboId);

    // set attrib arrays with stride and offset
    int stride = sphere.getInterleavedStride();     // should be 32 bytes
    glVertexAttribPointer(attribVertex, 3, GL_FLOAT, false, stride, (void*)0);
    glVertexAttribPointer(attribNormal, 3, GL_FLOAT, false, stride, (void*)(sizeof(float) * 3));
    glVertexAttribPointer(attribTexCoord, 2, GL_FLOAT, false, stride, (void*)(sizeof(float) * 6));

    // activate attrib arrays
    glEnableVertexAttribArray(attribVertex);
    glEnableVertexAttribArray(attribNormal);
    glEnableVertexAttribArray(attribTexCoord);

    // draw a sphere with VBO
    glDrawElements(GL_TRIANGLES,
        sphere.getIndexCount(),
        GL_UNSIGNED_INT,
        (void*)0);
}

void drawCircle(GLfloat x, GLfloat y, GLfloat z, GLfloat radius)
{
    GLint numberOfVertices = 100;

    GLfloat doublePi = 2.0f * (3.14159265358979323846);

    GLfloat circleVerticesX[100];
    GLfloat circleVerticesY[100];
    GLfloat circleVerticesZ[100];

    //circleVerticesX[0] = x;
    //circleVerticesY[0] = y;
    //circleVerticesZ[0] = z;

    for (int i = 0; i < numberOfVertices; i++)
    {
        circleVerticesX[i] = x + (radius * cos(i * doublePi / 99));
        circleVerticesY[i] = y + (radius * sin(i * doublePi / 99));
        circleVerticesZ[i] = z;
    }

    GLfloat allCircleVertices[100 * 3];

    for (int i = 0; i < numberOfVertices; i++)
    {
        allCircleVertices[i * 3] = circleVerticesX[i];
        allCircleVertices[(i * 3) + 1] = circleVerticesY[i];
        allCircleVertices[(i * 3) + 2] = circleVerticesZ[i];
    }

    // vertex data VBO
    GLuint vboId;
    glGenBuffers(1, &vboId);
    glBindBuffer(GL_ARRAY_BUFFER, vboId);
    glBufferData(GL_ARRAY_BUFFER,
        sizeof(allCircleVertices),
        allCircleVertices,
        GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vboId);
    glVertexAttribPointer(0, 3, GL_FLOAT, false, 3 * sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(0);

    glDrawArrays(GL_LINE_STRIP, 0, numberOfVertices);
}

//void renderPlane()
//{
//    int attribVertex = 0, attribNormal = 1, attribTexCoord = 2;
//
//    GLfloat planeVertices[] = {
//        -50,50,0,
//        50,50,0,
//        50,-50,0,
//        -50,-50,0,
//        -50,50,0,
//    };
//
//    GLfloat planeVerticesNormal [] = {
//       0,0,-1,
//       0,0,-1,
//       0,0,-1,
//       0,0,-1,
//       0,0,-1,
//    };
//
//    // vertex data VBO
//    GLuint vboId;
//    glGenBuffers(1, &vboId);
//    glBindBuffer(GL_ARRAY_BUFFER, vboId);
//    glBufferData(GL_ARRAY_BUFFER,
//        sizeof(planeVertices),
//        planeVertices,
//        GL_STATIC_DRAW);
//
//    // bind
//    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GL_FLOAT), (void*)0);
//    glEnableVertexAttribArray(0);
//
//    glDrawArrays(GL_TRIANGLE_STRIP, 0, 5);
//}

// Get the ray from the mouse position
Ray getRayFromMouse(float mouseX, float mouseY, int windowWidth, int windowHeight, glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
    // Convert the mouse position to normalized device coordinates
    float ndcX = (2.0f * mouseX) / windowWidth - 1.0f;
    float ndcY = 1.0f - (2.0f * mouseY) / windowHeight;

    // Get the clip space position of the mouse
    glm::vec4 clipSpacePosition = glm::vec4(ndcX, ndcY, -1.0f, 1.0f);

    // Get the eye space position of the mouse
    glm::mat4 inverseProjectionMatrix = glm::inverse(projectionMatrix);
    glm::vec4 eyeSpacePosition = inverseProjectionMatrix * clipSpacePosition;
    eyeSpacePosition = glm::vec4(eyeSpacePosition.x, eyeSpacePosition.y, -1.0f, 0.0f);

    // Get the world space position of the mouse
    glm::mat4 inverseViewMatrix = glm::inverse(viewMatrix);
    glm::vec4 worldSpacePosition = inverseViewMatrix * eyeSpacePosition;
    glm::vec3 rayOrigin = glm::vec3(worldSpacePosition);

    // Get the direction of the ray
    glm::vec3 rayDirection = glm::normalize(rayOrigin - glm::vec3(viewMatrix[3]));

    return Ray{ rayOrigin, rayDirection };
}

bool intersectRayPoint(Ray ray, glm::vec3 point, float& t) {
    // Calculate the direction vector from the ray origin to the point
    glm::vec3 direction = point - ray.origin;

    // Check if the direction vector is parallel to the ray direction
    float ndotv = glm::dot(direction, ray.direction);
    if (glm::abs(ndotv) < 0.0001f) {
        return false;
    }

    // Calculate the distance from the ray origin to the point
    t = glm::dot(direction, ray.direction) / glm::dot(ray.direction, ray.direction);

    if (t < 0.0f) {
        return false;
    }

    return true;
}