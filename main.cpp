#include <iostream>
#include <cmath> 
#include <vector> 

using namespace std;

class Vector3D{
	double x;
	double y;
	double z;
public:
	Vector3D()
	{

	}
	Vector3D(double x1, double y1, double z1)
	{
		x = x1;
		y = y1;
		z = z1;
	}
	double get_x()
	{
		return x;
	}
	double get_y()
	{
		return y;
	}
	double get_z()
	{
		return z;
	}
	double norm()
	{
		return sqrt(x*x + y*y + z*z);
	}
	Vector3D operator - (Vector3D a)
	{
		return Vector3D(x - a.get_x(), y - a.get_y(), z - a.get_z());
	}
	void operator = (Vector3D a)
	{
		x = a.get_x();
		y = a.get_y();
		z = a.get_z();
	}
};

class Segment3D
{
	Vector3D start, end;
public:
	Segment3D(Vector3D s, Vector3D e)
	{
		start = s;
		end = e;
	}
	Vector3D DirectiveWay()
	{
		return Vector3D(end.get_x() - start.get_x(), end.get_y() - start.get_y(), end.get_z() - start.get_z());
	}
	Vector3D get_start()
	{
		return start;
	}
	Vector3D get_end()
	{
		return end;
	}
	Vector3D paramPoint(double t)
	{
	    double a1 = start.get_x() - end.get_x();
		double a2 = end.get_x();
		double b1 = start.get_y() - end.get_y();
		double b2 = end.get_y();
		double c1 = start.get_z() - end.get_z();
		double c2 = end.get_z();
		return Vector3D(a1*t + a2, b1*t + b2, c1*t + c2);
	}
};

double dot(Vector3D a, Vector3D b)
{
	return a.get_x()*b.get_x() + a.get_y()*b.get_y() + a.get_z()*b.get_z();
}
double MixedProduct(Vector3D a, Vector3D b, Vector3D c)
{
	return (a.get_x()*b.get_y()*c.get_z() + a.get_z()*b.get_x()*c.get_y() + a.get_y()*b.get_z()*c.get_x() - a.get_z()*b.get_y()*c.get_x() - a.get_x()*b.get_z()*c.get_y() - a.get_y()*b.get_x()*c.get_z());
}

Vector3D intersect(Segment3D I1, Segment3D I2)
{
	double eps = 0.001;

	try
	{
		if (abs(MixedProduct(I1.DirectiveWay(), I2.DirectiveWay(), I1.get_start() - I2.get_start())) < eps)
		{
			Vector3D V1=I2.get_start()-I1.get_start();
			Vector3D V2=I2.get_end()-I1.get_start();
			Vector3D V3 = I1.DirectiveWay();
			
			double alpha1 = acos(dot(V3, V1) / (V1.norm()*V3.norm()));
			double alpha2 = acos(dot(V3, V2) / (V3.norm()*V2.norm()));
			double alpha3 = acos(dot(V1, V2) / (V1.norm()*V2.norm()));
            //Проверяем, что направляющий вектор лужит внутри области, образованной V1 и V2
			if (alpha1 <= alpha3 && alpha2 <= alpha3)
			{
			    //Проверяем, что точка не лежит на отрезке 
				if (V1.norm() + V2.norm()>I2.DirectiveWay().norm())
				{
				    //высоты 
				    double h1=V1.norm()*sin(alpha1);
				    double h2=V2.norm()*sin(alpha2);
				    //Считаем расстояние, которое нужно пройти до пересечения
				    double d_opt;
					if (V1.norm()*cos(alpha1)<V2.norm()*cos(alpha2))
					    d_opt=V1.norm()*cos(alpha1)+h1/(h2+h1)*(V2.norm()*cos(alpha2)-V1.norm()*cos(alpha1));
					else 
					    d_opt=V2.norm()*cos(alpha2)+h2/(h2+h1)*(V1.norm()*cos(alpha1)-V2.norm()*cos(alpha2));
					//Если больше, чем длина самого отрезка, то нет пересчения, иначе находим точку
					if (V3.norm() > d_opt)
					{
						double t_opt =1 - (d_opt / V3.norm());
						return I1.paramPoint(t_opt);
					}
					else 
					throw "Segments do not intersect";
				}
				else
				    return Vector3D(I1.get_start().get_x(),I1.get_start().get_y(),I1.get_start().get_z());
			}
			else
				throw "Segments do not intersect";
		}
		throw "Segments do not lie in the same plane";
	}
	catch (const char* exception)
	{
		cerr << exception << endl;
	}
}
int main()
{
	Segment3D I1(Vector3D(0.5, 0.5, 0.5), Vector3D(0.51, 0.51, 0.49));
	Segment3D I2(Vector3D(0.78, 0.22,0.5), Vector3D(0.49, 0.51, 0.5));
	Vector3D A = intersect(I2, I1);
	cout << A.get_x() << " " << A.get_y() << " " << A.get_z() << endl<<endl;
	
	Segment3D I3( Vector3D(7, -7, -6),Vector3D(9, -9, -10));
	Segment3D I4(Vector3D(5, -4, -14),Vector3D(11, -12, -2));
	Vector3D B = intersect(I4, I3);
	cout << endl<<B.get_x() << " " << B.get_y() << " " << B.get_z() << endl<<endl;
	
	Segment3D I5( Vector3D(7, 4, 9),Vector3D(-2, 1, -3));
	Segment3D I6(Vector3D(7, 4, 6),Vector3D(-2, 1, 3));
	Vector3D C = intersect(I5, I6);
	cout << endl<<C.get_x() << " " << C.get_y() << " " << C.get_z() << endl<<endl;
	
	Segment3D I7( Vector3D(-3, 0, -2),Vector3D(0, 9, -11));
	Segment3D I8(Vector3D(-6, 3, -15),Vector3D(0, 3, 0));
	Vector3D D = intersect(I8, I7);
	cout << endl<<D.get_x() << " " << D.get_y() << " " << D.get_z() << endl<<endl;
	
	Segment3D I9( Vector3D(-1, 9, 8),Vector3D(1, 5, 2));
	Segment3D I10(Vector3D(-2, 11, 6),Vector3D(2, 3, 4));
	Vector3D E = intersect(I9, I10);
	cout << endl<<E.get_x() << " " << E.get_y() << " " << E.get_z() << endl<<endl;
	return 0;
}
