//
//  Shader.fsh
//  MyGLGame
//
//  Created by Borna Noureddin on 2015-01-28.
//  Copyright (c) 2015 BCIT. All rights reserved.
//

varying lowp vec4 colorVarying;

void main()
{
    gl_FragColor = colorVarying;
}
