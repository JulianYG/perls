// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW. Always include it before gl.h and glfw.h, since it's a bit magic.
// #include <OpenGL/glew.h>
// #include <OpenGL/glfw3.h>
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

// #include <glm/glm.hpp>
// using namespace glm;

// GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0};  /* Red diffuse light. */
// GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};  /* Infinite light location. */
// GLfloat n[6][3] = {  /* Normals for the 6 faces of a cube. */
//   {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
//   {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0} };
// GLint faces[6][4] = {  /* Vertex indices for the 6 faces of a cube. */
//   {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
//   {4, 5, 1, 0}, {5, 6, 2, 1}, {7, 4, 0, 3} };
// GLfloat v[8][3];  /* Will be filled in with X,Y,Z vertexes. */

// void
// drawBox(void)
// {
//   int i;

//   for (i = 0; i < 6; i++) {
//     glBegin(GL_QUADS);
//     glNormal3fv(&n[i][0]);
//     glVertex3fv(&v[faces[i][0]][0]);
//     glVertex3fv(&v[faces[i][1]][0]);
//     glVertex3fv(&v[faces[i][2]][0]);
//     glVertex3fv(&v[faces[i][3]][0]);
//     glEnd();
//   }
// }

// void
// display(void)
// {
//   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//   drawBox();
//   glutSwapBuffers();
// }

// void
// init(void)
// {
//   /* Setup cube vertex data. */
//   v[0][0] = v[1][0] = v[2][0] = v[3][0] = -1;
//   v[4][0] = v[5][0] = v[6][0] = v[7][0] = 1;
//   v[0][1] = v[1][1] = v[4][1] = v[5][1] = -1;
//   v[2][1] = v[3][1] = v[6][1] = v[7][1] = 1;
//   v[0][2] = v[3][2] = v[4][2] = v[7][2] = 1;
//   v[1][2] = v[2][2] = v[5][2] = v[6][2] = -1;

//   /* Enable a single OpenGL light. */
//   glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
//   glLightfv(GL_LIGHT0, GL_POSITION, light_position);
//   glEnable(GL_LIGHT0);
//   glEnable(GL_LIGHTING);

//   /* Use depth buffering for hidden surface elimination. */
//   glEnable(GL_DEPTH_TEST);

//   /* Setup the view of the cube. */
//   glMatrixMode(GL_PROJECTION);
//   gluPerspective( /* field of view in degree */ 40.0,
//     /* aspect ratio */ 1.0,
//     /* Z near */ 1.0, /* Z far */ 10.0);
//   glMatrixMode(GL_MODELVIEW);
//   gluLookAt(0.0, 0.0, 5.0,  /* eye is at (0,0,5) */
//     0.0, 0.0, 0.0,      /* center is at (0,0,0) */
//     0.0, 1.0, 0.);      /* up is in positive Y direction */

//   /* Adjust cube position to be asthetic angle. */
//   glTranslatef(0.0, 0.0, -1.0);
//   glRotatef(60, 1.0, 0.0, 0.0);
//   glRotatef(-20, 0.0, 0.0, 1.0);
// }

GLuint quad_VertexArrayID;
static const GLfloat g_quad_vertex_buffer_data[] = {
    -1.0f, -1.0f, 0.0f,
    1.0f, -1.0f, 0.0f,
    -1.0f,  1.0f, 0.0f,
    -1.0f,  1.0f, 0.0f,
    1.0f, -1.0f, 0.0f,
    1.0f,  1.0f, 0.0f,
};

GLuint quad_vertexbuffer;
// Create and compile our GLSL program from the shaders
GLuint quad_programID = LoadShaders( "Passthrough.vertexshader", "SimpleTexture.fragmentshader" );
GLuint texID = glGetUniformLocation(quad_programID, "renderedTexture");
GLuint timeID = glGetUniformLocation(quad_programID, "time");



bool 
createRenderTarget(void)
{
    GLuint FramebufferName = 0;
    glGenFramebuffers(1, &FramebufferName);
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

    GLuint renderedTexture;
    glGenTextures(1, &renderedTexture);

    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_2D, renderedTexture);

    // Give an empty image to OpenGL ( the last "0" )
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, 1024, 768, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);

    // Poor filtering. Needed !
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    GLuint depthrenderbuffer;
    glGenRenderbuffers(1, &depthrenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, 1024, 768);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);

    // Set "renderedTexture" as our colour attachement #0
    glFramebufferTextureEXT(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

    // Set the list of draw buffers.
    GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

    // Always check that our framebuffer is ok
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        return false;
    return true;
}

bool
renderingToTexture(void)
{
    // Render to our framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
    glViewport(0,0,1024,768); // Render on the whole framebuffer, complete from the lower left corner to the upper right
    layout(location = 0);
}

void 
renderingToScreen(void)
{
    // The fullscreen quad's FBO
    GLuint quad_VertexArrayID;
    glGenVertexArrays(1, &quad_VertexArrayID);
    glBindVertexArray(quad_VertexArrayID);

    static const GLfloat g_quad_vertex_buffer_data[] = {
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f,  1.0f, 0.0f,
    };

    GLuint quad_vertexbuffer;
    glGenBuffers(1, &quad_vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);

    // Create and compile our GLSL program from the shaders
    GLuint quad_programID = LoadShaders( "Passthrough.vertexshader", "SimpleTexture.fragmentshader" );
    GLuint texID = glGetUniformLocation(quad_programID, "renderedTexture");
    GLuint timeID = glGetUniformLocation(quad_programID, "time");
    // Render to the screen
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0,0,1024,768); // Render on the whole framebuffer, complete from the lower left corner to the upper right
}

in vec2 UV;

out vec3 color;

uniform sampler2D renderedTexture;
uniform float time;

int
main()
{
    color = texture( renderedTexture, UV + 0.005*vec2( sin(time+1024.0*UV.x),cos(time+768.0*UV.y)) ).xyz;
}




