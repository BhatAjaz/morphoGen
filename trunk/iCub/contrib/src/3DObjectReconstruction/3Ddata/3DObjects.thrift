namespace yarp yarp.sig

struct PointXYZ {
1: double x;
2: double y;
3: double z;
}

struct Polygon{
1: list<i32> vertices;
}

struct RGBA{
1: i32 rgba
}


struct SurfaceMesh{ 
1: string meshName
2: list<PointXYZ> points;
3: optional list<RGBA> rgbColour;
4: optional list<Polygon> polygons;
}
