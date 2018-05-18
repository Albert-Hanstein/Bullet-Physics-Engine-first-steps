#version 400

in  vec4 aPosition;
uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uModel;


void main(void) {
    mat4 MVP = uProjection * uView * uModel;
    gl_Position = MVP * aPosition;
    }
