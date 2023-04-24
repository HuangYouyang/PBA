#version 330 core
layout(triangles) in;
layout(line_strip, max_vertices=4) out;

void main()
{
    gl_Position = gl_in[0].gl_Position + vec4(0,0.0,0,0.1);
    EmitVertex();
    gl_Position = gl_in[1].gl_Position + vec4(0,0.0,0,0.1);
    EmitVertex();
    gl_Position = gl_in[2].gl_Position + vec4(0,0.0,0,0.1);
    EmitVertex();
    gl_Position = gl_in[0].gl_Position + vec4(0,0.0,0,0.1);
    EmitVertex();
    EndPrimitive();
}