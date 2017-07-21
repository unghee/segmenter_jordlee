// handle to texture
uniform sampler2D frame;

// kernels for edge detection (to load from program)
uniform mat3 mSobelX;
uniform mat3 mSobelY;

// Threshold for removing noise
uniform float fThreshold;
uniform bool binary = false;
uniform bool norm = false;

vec4 sobel()
{
  vec3 vNormal, vCenterNormal;
  vec2 vG = vec2(0.0,0.0);
  float fGm;
  vec4 vNull;
  vec4 vResult;

  if( binary )
    vNull = vec4(0.0, 0.0, 0.0, 0.0);
  else
    vNull = vec4(0.5, 0.5, 0.0, 0.0);

  vCenterNormal = texture2D(frame, gl_TexCoord[0].st).xyz;

  // convolute sobel kernel for each color channel
  for( int i = 0; i < 3; i++ ) {
    for( int j = 0; j < 3; j++ ) {
      vNormal = textureOffset(frame, gl_TexCoord[0].st, ivec2((i - 1), (j - 1))).xyz;
      float dp = dot(vCenterNormal, vNormal);
      vG.x += mSobelX[i][j] * dp;
      vG.y += mSobelY[i][j] * dp;
    }
  }

  // magnitude of edge
  fGm = length(vG);

  // Threshold for removing background noise
  if( fGm <= fThreshold ) {
    vResult = vNull;
  } else {

    if( binary ) {
      if( norm )
        vResult = vec4(1.0f, 1.0f, 1.0f, 0.0);
      else
        vResult = vec4(fGm, fGm, fGm, 0.0);
    } else {

      if( norm )
        vG = normalize(vG);

      // scale gradient to range = [0 ... 1]
      vG = vG * 0.5 + 0.5;
      vResult = vec4(vG, fGm, 0.0);

    }
  }

  return vResult;
}

void main()
{
  gl_FragColor = sobel();
}
