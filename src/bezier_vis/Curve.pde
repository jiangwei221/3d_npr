float scale_fac = 100;
class Curve
{
  ArrayList<PVector> control_pts;
  PVector colors;
  Curve(String data[])
  {
    control_pts = new ArrayList<PVector>();
    for(int i=0; i<data.length; i=i+3)
    {
      control_pts.add(new PVector(Float.parseFloat(data[i]), Float.parseFloat(data[i+1]), Float.parseFloat(data[i+2])).mult(scale_fac));
    }
    colors = new PVector(random(0,255),random(0,255),random(0,255));
  }
  
  void draw()
  {
    for(int i=0;i<control_pts.size()-1; i+=3)
    {
      for(float t=0; t<=1.0; t+=1.0/10)
      {
        float x1 = control_pts.get(i).x;
        float y1 = control_pts.get(i).y;
        float z1 = control_pts.get(i).z;
        float x2 = control_pts.get(i+1).x;
        float y2 = control_pts.get(i+1).y;
        float z2 = control_pts.get(i+1).z;
        float x3 = control_pts.get(i+2).x;
        float y3 = control_pts.get(i+2).y;
        float z3 = control_pts.get(i+2).z;
        float x4 = control_pts.get(i+3).x;
        float y4 = control_pts.get(i+3).y;
        float z4 = control_pts.get(i+3).z;
        
        //pushMatrix();
        //translate(x1, y1, z1);
        //sphere(1);
        //popMatrix();
        //pushMatrix();
        //translate(x2, y2, z2);
        //sphere(1);
        //popMatrix();
        //pushMatrix();
        //translate(x3, y3, z3);
        //sphere(1);
        //popMatrix();
        //pushMatrix();
        //translate(x4, y4, z4);
        //sphere(1);
        //popMatrix();
        //println(z4);
        noFill();
        stroke(colors.x, colors.y, colors.z);
        bezier(x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4);
        //float x = bezierPoint(x1, x2, x3, x4, t);
        //float y = bezierPoint(y1, y2, y3, y4, t);
        //float z = bezierPoint(z1, z2, z3, z4, t);
        //pushMatrix();
        //translate(x, y, z);
        ////println(x,y,z);
        //sphere(1);
        //popMatrix();

      }
    }
  }
}
