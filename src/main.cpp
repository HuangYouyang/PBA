#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>

#include "shader.h"
#include "camera.h"
#include "mesh.h"
#include "model.h"
#include "utils.h"
#include "light.h"
#include "Sphere.h"
#include "Line.h"

unsigned int depthMapFBO, depthMap;
static const int SHADOW_WIDTH = 800, SHADOW_HEIGHT = 600;
bool model_draw = true, display_corner = false, move_light = false;

// mouse control
bool leftButton = false;
float startX, startY, startZ;
Arrow arrow = Arrow();

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
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

int main()
{
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
    // Shader lightingShader("shaders/cursor/cursor.vert", "shaders/cursor/cursor.frag", shaders/cursor/cursor.geom");
    Shader lightingShader("shaders/shadow/shadow.vert", "shaders/shadow/shadow.frag");
    //Shader depthShader("shaders/depth/depth.vert", "shaders/depth/depth.frag");
    //Shader cornerShader("shaders/corner/corner.vert", "shaders/corner/corner.frag");

    // ------------------------------------------------------------------
    // Corner : set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    /*unsigned int quadVBO, quadVAO;
    float corner[] = {
         0.5f,  1.0f,  0.0f, 1.0f,
         0.5f,  0.5f,  0.0f, 0.0f,
         1.0f,  0.5f,  1.0f, 0.0f,
         0.5f,  1.0f,  0.0f, 1.0f,
         1.0f,  0.5f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };
    float quad[] = {
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);


    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), &quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    unsigned int cornerVAO, cornerVBO;
    glGenVertexArrays(1, &cornerVAO);
    glGenBuffers(1, &cornerVBO);
    glBindVertexArray(cornerVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cornerVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(corner), &corner, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);*/

    // load models 
    // Model bunny("assets/bunny/bunny.obj");
    Light lights(LightPositions, 4);

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

        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 1.0f, 100.0f);
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 tmpmodel = glm::scale(model, glm::vec3(scale, scale, scale));
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 lightSpaceTrans = glm::lookAt(lightPos, glm::vec3(0.0f), camera.WorldUp);
        const auto renderScene = [&](Shader& shader)
        {
            renderSphere();

            // translate and render another
            model = glm::translate(model, glm::vec3(0.0f,0.5f,0.0f));
            shader.setMat4("model", model);
            renderArrow();
            
            // translate and render another

            // drag left mouse to force the particle
            if (leftButton) {
                //getting cursor position
               double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);

                arrow.draw(xpos, ypos, 0);

                /*preX = xpos;
                preY = ypos;*/
            }
            // render 3D sphere
            //renderSphere();
        };
        /*if (display_corner) {
            glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
            glEnable(GL_DEPTH_TEST);
            glClearColor(0.5f, 0.5f, 0.5f, 0.5f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            depthShader.use();

            // view/projection transformations
            model = glm::mat4(1.0f);

            // depthShader.setMat4("projection",projection);
            depthShader.setMat4("projection", glm::perspective(glm::radians(89.0f), (float)SHADOW_WIDTH / SHADOW_HEIGHT, 0.1f, 10.0f));
            depthShader.setMat4("view", lightSpaceTrans);
            depthShader.setMat4("model", model);
            depthShader.setVec3("viewPos", lightPos);
            renderScene(depthShader);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            model = glm::mat4(1.0f);
        }*/

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

        /*if (display_corner) {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glDisable(GL_DEPTH_TEST);
            cornerShader.use();
            glBindVertexArray(cornerVAO);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, depthMap);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }*/

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // ------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------
    /*glDeleteVertexArrays(1, &quadVAO);
    glDeleteBuffers(1, &quadVBO);*/

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

void gen_preview_framebuffer() {
    glGenFramebuffers(1, &depthMapFBO);
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
        SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
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
    //if (firstMouse)
    //{
    //    lastX = xpos;
    //    lastY = ypos;
    //    firstMouse = false;
    //}

    //float xoffset = xpos - lastX;
    //float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    //lastX = xpos;
    //lastY = ypos;

    //camera.ProcessMouseMovement(xoffset, yoffset);

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
            arrow.setStart(xpos, ypos, 0);
        }
        else
            leftButton = false;
    }
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
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
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void renderArrow()
{
    int attribVertex = 0, attribNormal = 1, attribTexCoord = 2;

    GLfloat lineVertices[] = {
        10, 0, 0,
        100, 300, 0
    };

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