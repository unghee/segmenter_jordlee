
uniform sampler2D tex1;
uniform sampler2D tex2;

uniform bool tex1grad = false;
uniform bool tex2grad = false;
uniform bool byfactor = false;
uniform float k;

void main()
{
  vec4 vColor1 = texture2D(tex1, gl_TexCoord[0].st);
  vec4 vColor2 = texture2D(tex2, gl_TexCoord[0].st);

  if(byfactor){

    gl_FragColor = vColor1 * k;

  }else{

    if(tex1grad)
      vColor1 = (vColor1 - 0.5f) * 2.0f;
    if(tex2grad)
      vColor2 = (vColor2 - 0.5f) * 2.0f;

    vec4 vColor = vec4( vColor1.r * vColor2.r,
                        vColor1.g * vColor2.g,
                        vColor1.b * vColor2.b,
                        vColor1.a * vColor2.a);

    if(tex1grad || tex2grad){
      float fGm = length(vColor.rg);
      vColor = vColor * 0.5f + 0.5f;
      vColor.b = fGm;
      vColor.a = 1.0;
    }

    gl_FragColor = vColor;

  }
}
