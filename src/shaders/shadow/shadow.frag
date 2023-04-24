#version 330 core
layout(location=0) out vec4 FragColor;
layout(location=1) out vec4 Color;

struct Material{
    sampler2D diffuse;
    sampler2D specular;
    float shininess;
};
in vec3 Normal;
in vec3 FragPos;
in vec2 TexCoords;
in vec4 FragPosLightSpace;

uniform vec3 lightPos;
uniform vec3 objectColor;
uniform vec3 lightColor;
uniform vec3 viewPos;
uniform Material material;
uniform sampler2D shadowMap;

uniform vec3 ArrowColor;
uniform int flag=1;

float ShadowCalc(){
    vec3 projCoords = FragPosLightSpace.xyz / FragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;
    if(projCoords.x<0.0||projCoords.x>1.0||projCoords.y<0.0||projCoords.y>1.0)
        return 1.0;
    float depth = texture(shadowMap,projCoords.xy).r;
    float currentDepth = projCoords.z;
    float shadow = currentDepth < depth+0.01? 1.0:0.0;
    return shadow;
}

void main()
{
    float ambientStrength = 1.0;
    float specularStrength = 0.5;

    vec3 ambient= ambientStrength * objectColor;
    
    vec3 norm = normalize(Normal);
    vec3 viewDir= normalize(viewPos-FragPos);
    vec3 lightDir = normalize(lightPos-FragPos);
    vec3 reflectDir = reflect(-lightDir,norm);
    float diff = max(dot(norm,lightDir),0.0);
    vec3 diffuse = diff * objectColor;

    float spec = pow(max(dot(viewDir,reflectDir),0.0),material.shininess);
    vec3 specular = specularStrength * spec * lightColor;
    //vec3 result = (ambient + diffuse) * texture(material.diffuse,TexCoords).rgb+specular*texture(material.specular,TexCoords).rgb;

    float shadow = ShadowCalc();
    vec3 result = (ambient +(shadow)* diffuse) * texture(material.diffuse,TexCoords).rgb+(shadow)*specular*texture(material.specular,TexCoords).rgb;
    //FragColor = vec4(result, 1.0);

    if(flag == 1)
        FragColor = vec4(ambient + specular + diffuse, 1.0);
    else if(flag == 2)
        FragColor = vec4(ArrowColor, 1.0);
}
