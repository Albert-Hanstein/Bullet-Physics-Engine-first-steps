#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

/*
*
* This is aheader file for some useful code.
*
*/
/*
 *
 * Include files for Windows, Linux and OSX
 * __APPLE is defined if OSX, otherwise Windows and Linux.
 *
 */

#ifdef __APPLE__
#ifndef GLFW_INCLUDE_GLCOREARB
#define GLFW_INCLUDE_GLCOREARB 1
#endif
#include <GLFW/glfw3.h>
#else
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <map>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <assert.h>

void Check(const char *);

char* filetobuf(char *file);

void CheckShader(int sp, const char *x); // New to check shader program

/*
    Helper routines
*/

void print(glm::vec2 x);
void print(glm::mat2 x);
void print(glm::vec3 x);
void print(glm::mat3 x);
void print(glm::vec4 x);
void print(glm::mat4 x);
void print(const char *x);
void print(float x);
void print(int x);

void Print(glm::vec2 x);
void Print(glm::mat2 x);
void Print(glm::vec3 x);
void Print(glm::mat3 x);
void Print(glm::vec4 x);
void Print(glm::mat4 x);
void Print(const char *x);
void Print(float x);
void Print(int x);
void Print(void);

void delay(unsigned long ms);

char* filetobuf(char *file);

#endif // UTILS_H_INCLUDED
