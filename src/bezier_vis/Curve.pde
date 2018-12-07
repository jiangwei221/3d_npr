float scale_fac = 100;
class Curve
{
  ArrayList<PVector> control_pts;
  ArrayList<PVector> v_list;
  ArrayList<PVector> f_list;//every 3 is a face, it is NOT index
  PVector colors;
  Curve(String data[])
  {
    control_pts = new ArrayList<PVector>();
    v_list = new ArrayList<PVector>();
    f_list = new ArrayList<PVector>();
    for(int i=0; i<data.length; i=i+3)
    {
      control_pts.add(new PVector(Float.parseFloat(data[i]), Float.parseFloat(data[i+1]), Float.parseFloat(data[i+2])).mult(scale_fac));
    }
    colors = new PVector(random(0,255),random(0,255),random(0,255));
    generate_bz_mesh();
  }
  
  void draw()
  {
    for(int i=0;i<control_pts.size()-1; i+=3)
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
      
      noFill();
      stroke(colors.x, colors.y, colors.z);
      bezier(x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4);
    }
  }
  
  void generate_bz_mesh()
  {
    ArrayList<PVector> pt_list = new ArrayList<PVector>();
    for(int i=0;i<control_pts.size()-1; i+=3)
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
      for(float t=0; t<=1.0; t+=1.0/10.0)
      {
        float x = bezierPoint(x1, x2, x3, x4, t);
        float y = bezierPoint(y1, y2, y3, y4, t);
        float z = bezierPoint(z1, z2, z3, z4, t);
        pt_list.add(new PVector(x,y,z));
      }
    }
    //now, we have all the intermedia points
    for(int i=1; i<=pt_list.size()-2; i++)
    {
      PVector pre = pt_list.get(i-1);
      PVector cur = pt_list.get(i);
      PVector next = pt_list.get(i+1);
      PVector norm = PVector.sub(cur, pre).cross(PVector.sub(next, cur));
      norm.normalize();
      norm.mult(3);
      if(i==1)
      {
        v_list.add(PVector.sub(pt_list.get(0), norm));
        //v_list.add(pt_list.get(0));
        v_list.add(PVector.add(pt_list.get(0), norm));
      }
      v_list.add(PVector.sub(pt_list.get(i), norm));
      //v_list.add(pt_list.get(i));
      v_list.add(PVector.add(pt_list.get(i), norm));
      if(i==pt_list.size()-2)
      {
        v_list.add(PVector.sub(pt_list.get(pt_list.size()-1), norm));
        //v_list.add(pt_list.get(pt_list.size()-1));
        v_list.add(PVector.add(pt_list.get(pt_list.size()-1), norm));
      }
    }
    
    //now, we have all the v for the mesh
    for(int i=0;i<v_list.size()-3;i=i+2)
    {
      PVector left = v_list.get(i);
      PVector right = v_list.get(i+1);
      PVector left_next = v_list.get(i+2);
      PVector right_next = v_list.get(i+3);
      //front
      f_list.add(left);
      f_list.add(right);
      f_list.add(right_next);
      
      f_list.add(left);
      f_list.add(right_next);
      f_list.add(left_next);
      //back
      f_list.add(left);
      f_list.add(right_next);
      f_list.add(right);
      
      f_list.add(left);
      f_list.add(left_next);
      f_list.add(right_next);
    }
  }
  
  void draw_mesh()
  {
    for(int i=0;i<f_list.size()-2;i++)
    {
      fill(colors.x, colors.y, colors.z);
      beginShape();
      vertex(f_list.get(i).x, f_list.get(i).y, f_list.get(i).z);
      vertex(f_list.get(i+1).x, f_list.get(i+1).y, f_list.get(i+1).z);
      vertex(f_list.get(i+2).x, f_list.get(i+2).y, f_list.get(i+2).z);
      endShape(CLOSE);
    }
  }
}
