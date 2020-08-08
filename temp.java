public static Vector3D Map2DTo3D(this Intrinsics intrinsics, Vector2D pixel, float depth)
{
    Vector3D point = new Vector3D();

    float x = (pixel.X - intrinsics.ppx) / intrinsics.fx;
    float y = (pixel.Y - intrinsics.ppy) / intrinsics.fy;

    point.X = depth * x;
    point.Y = depth * y;
    point.Z = depth;

    return point;
}