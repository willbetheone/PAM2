attribute vec4 aPosition;
uniform mat4 uModelViewProjectionMatrix;

invariant gl_Position;

void main()
{
    gl_Position = uModelViewProjectionMatrix * aPosition;
}