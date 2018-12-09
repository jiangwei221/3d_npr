/*
read bezier curve from csv
and render it

jiang wei
uvic
start: 2018.10.10
*/
import peasy.*;

PeasyCam cam;

ArrayList<Curve>curves;
PShape s;

void setup()
{
  
  
  cam = new PeasyCam(this, 0,0,0,1000);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(500);
  
  size(800,800, P3D);
  curves = new ArrayList<Curve>();
  String[] lines = loadStrings("test.txt");
  String[] color_lines = loadStrings("test_color.txt");
  //println(lines[0]);
  for (int row = 0; row < lines.length; row++)
  {
    String[] data = splitTokens(lines[row], ", []");
    String[] in_colors = splitTokens(color_lines[row], ", []");
    if(data.length>0)
      curves.add(new Curve(data, in_colors));
  }
  s = loadShape("monkey_v.obj");
  s.scale(99);
  s.setFill(color(100, 100, 120, 50));
}

void draw()
{
  background(128);
  //line(-100, 0, 0, 100, 0, 0);
  //line(0, -100, 0, 0, 100, 0);
  //line(0, 0, -100, 0, 0, 100);
  for(int i=0; i<curves.size(); i++ )
  //for(int i=0; i<10; i++ )
  {
    //curves.get(i).draw();
    //curves.get(i).draw();
    curves.get(i).draw_mesh();
  }
  pushMatrix();
  
  //noFill();
  
  //shape(s, 0, 0);
  popMatrix();
}
