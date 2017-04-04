//
//  Shader.vsh
//  MyGLGame
//
//  Created by Borna Noureddin on 2015-01-28.
//  Copyright (c) 2015 BCIT. All rights reserved.
//

attribute vec4 position;
attribute vec4 inColor;

varying lowp vec4 colorVarying;

uniform mat4 modelViewProjectionMatrix;

void main()
{
    colorVarying = inColor;
    
    gl_Position = modelViewProjectionMatrix * position;
}
