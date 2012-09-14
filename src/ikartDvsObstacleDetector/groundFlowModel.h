/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef GROUND_FLOW_MODEL_H
#define GROUND_FLOW_MODEL_H

#include <math.h>
#include <iCub/ctrl/math.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/math/SVD.h>
#include <string>

using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265
#endif

//#define N_PIXELS 64 
#define N_PIXELS 5

#ifndef IMGFOR
#define IMGFOR(img,i,j) for (int i=0; i<(img).width(); i++) for (int j=0; j<(img).height(); j++)
#endif

class groundFlowModel
{
	private:
	double input_grid_matrix_y [N_PIXELS][N_PIXELS];
	double input_grid_matrix_x [N_PIXELS][N_PIXELS];

	double input_ground_model_y [N_PIXELS][N_PIXELS];
	double input_ground_model_x [N_PIXELS][N_PIXELS];
	double output_ground_model_y [N_PIXELS][N_PIXELS];
	double output_ground_model_x [N_PIXELS][N_PIXELS];

	Matrix tx_matrix;
	Matrix p_matrix;

	public:
	yarp::sig::ImageOf<yarp::sig::PixelMono16> flow_model_image;

	void redraw()
	{	
		static const yarp::sig::PixelMono16 black=0;
        static const yarp::sig::PixelMono16 white=255;

		IMGFOR(flow_model_image ,i , j)
		{
			flow_model_image(i, j) = 150;
		}

		for (int x=0, X=0; x<N_PIXELS*4; x+=4, X++)
			for (int y=0, Y=0; y<N_PIXELS*4; y+=4, Y++)
			{
				//yarp::sig::draw::addSegment(flow_model_image,black,x,y,x+20,y+20);
				yarp::sig::draw::addSegment(flow_model_image,black,x,y,int(x+output_ground_model_x[X][Y]),int(y+output_ground_model_y[X][Y]));
				yarp::sig::draw::addCircle(flow_model_image,black,x+int(output_ground_model_x[X][Y]),int(y+output_ground_model_y[X][Y]),2);
			}
	}

	void project_plane()
	{
		Vector plane_vector(4);
		plane_vector[0] = 0;
		plane_vector[1] = 0;
		plane_vector[2] = 1;
		plane_vector[3] = 1;

		Vector n(3);
		n[0]=plane_vector[0];
		n[1]=plane_vector[1];
		n[2]=plane_vector[2];

		Vector orig(4,0.0);
		orig[0] = 0;
		orig[1] = 0;
		orig[2] = 0;
		orig[3] = 1;

		double z = 1.0;
		int u = 0;
		int v = 0;
		Matrix invPrj = Matrix(yarp::math::pinv(p_matrix.transposed()).transposed());
		for (u=0; u< N_PIXELS; u++)
			for (v=0; v< N_PIXELS; v++)
			{
				Vector p(3);
				p[0]=z*u;
				p[1]=z*v;
				p[2]=z;

				Vector xe=invPrj*p;
				xe[3]=1.0;  // impose homogeneous coordinates
				//printf ("xe %d %d >>> %s\n",u,v,xe.toString().c_str());

				Vector ray=(iCub::ctrl::SE3inv(tx_matrix)*xe).subVector(0,2);
				//printf ("ray %d %d >>> %s\n",u,v,ray.toString().c_str());

				Vector p0(3,0.0);
				/*
				if (plane_vector[0]!=0.0)
					p0[0]=-plane_vector[3]/plane_vector[0];
				else if (plane_vector[1]!=0.0)
					p0[1]=-plane_vector[3]/plane_vector[1];
				else if (plane_vector[2]!=0.0)
					p0[2]=-plane_vector[3]/plane_vector[2];
				*/
				//printf ("p0 %d %d >>> %s\n",u,v,p0.toString().c_str());

				Vector e=(iCub::ctrl::SE3inv(tx_matrix)*orig).subVector(0,2); 

				//printf ("e %d %d >>> %s\n",u,v,e.toString().c_str());

				// compute the projection
				Vector vray=ray-e;
				//printf ("vray=ray-e %d %d >>> %s\n",u,v,vray.toString().c_str());
				//printf ("n %d %d >>> %s\n",u,v,n.toString().c_str());
				Vector test2 = p0-e;
				//printf ("p0-e %d %d >>> %s\n",u,v,test2.toString().c_str());

				Vector result=e+(dot(p0-e,n)/dot(vray,n))*vray;

				//double test = dot(vray,n);
				//printf ("dot %d %d >>> %f\n",u,v,test);
				//printf ("result %d %d >>> %s\n",u,v,result.toString().c_str());
				//printf ("r-----------------\n");
				input_grid_matrix_x[u][v] = result[0];
				input_grid_matrix_y[u][v] = result[1];
			}
			        
	}

	void initialize ()
	{
		flow_model_image.resize(4*N_PIXELS,4*N_PIXELS);

		double ang = -135.0/180.0*M_PI;
		tx_matrix.resize(4,4);
		tx_matrix.zero();
		/*
		tx = 1   0   0   x
		     0   c  -s   y
			 0   s   c   z
			 0   0   0   1
		*/
		//x rotation
		tx_matrix[0][0] = 1;
		tx_matrix[1][1] = cos(ang);
		tx_matrix[2][2] = cos(ang);
		tx_matrix[1][2] = -sin(ang);
		tx_matrix[2][1] = sin(ang);
		tx_matrix[0][3] = 0; //x
		tx_matrix[1][3] = 0; //y
		tx_matrix[2][3] = 1; //z
		tx_matrix[3][3] = 1; 
		tx_matrix = iCub::ctrl::SE3inv(tx_matrix);

		p_matrix.resize(3,4);
		p_matrix.zero();
		double f = 25000;
		double c = N_PIXELS/2;
		p_matrix[0][0] = f;
		p_matrix[0][2] = c;
		p_matrix[1][1] = f;
		p_matrix[1][2] = c;
		p_matrix[2][2] = 1;
		/*
		p = f 0 c 0
		    0 f c 0
			0 0 1 0
		*/
	}

	void set_movement(double x_vel, double y_vel, double t_vel)
	{
		for (int x=0; x<N_PIXELS; x++)
			for (int y=0; y<N_PIXELS; y++)
				input_ground_model_x[x][y]=x_vel;

		for (int x=0; x<N_PIXELS; x++)
			for (int y=0; y<N_PIXELS; y++)
				input_ground_model_y[x][y]=y_vel;
	}

	void compute_model()
	{
		for (int x=0; x<N_PIXELS; x++)
		{
			for (int y=0; y<N_PIXELS; y++)
			{
				Vector point_in1;
				Vector point_out1;
				Vector point_in2;
				Vector point_out2;
				point_in1.resize(4);
				point_in1[0]=input_grid_matrix_x[x][y]; 
				point_in1[1]=input_grid_matrix_y[x][y]; 
				point_in1[2]=0; 
				point_in1[3]=1; 
				point_in2.resize(4);
				point_in2[0]=point_in1[0]+input_ground_model_x[x][y]; 
				point_in2[1]=point_in1[1]+input_ground_model_y[x][y]; 
				point_in2[2]=0; 
				point_in2[3]=1; 

				//printf("in1 %s\n", point_in1.toString().c_str());
				//printf("%s\n", point_in2.toString().c_str());

				point_out1 = tx_matrix * point_in1;
				//printf("T1 %s\n", point_out1.toString().c_str());
				point_out1 = p_matrix  * point_out1;
				point_out1 = (1.0/point_out1[2]) * point_out1 ;
				//printf("P1 %s\n", point_out1.toString().c_str());
					
				//printf("in2 %s\n", point_in2.toString().c_str());
				point_out2 = tx_matrix * point_in2;
				//printf("T2 %s\n", point_out2.toString().c_str());
				point_out2 = p_matrix  * point_out2;
				point_out2 = (1.0/point_out2[2]) * point_out2 ;
				//printf("P2 %s\n\n", point_out2.toString().c_str());

				output_ground_model_x[x][y] = point_out2[0]-point_out1[0];
				output_ground_model_y[x][y] = point_out2[1]-point_out1[1];	
#if TEST_PROJ
				output_ground_model_x[x][y] = point_out2[0];
				output_ground_model_y[x][y] = point_out2[1];		
#endif 
			}
		}
	}
	groundFlowModel()
	{
		initialize();
		project_plane();
		set_movement (0.0, 1.0, 0.0);
		compute_model();

		printf ("----------\n");
		for (int x=0; x<N_PIXELS; x++)
		{
			for (int y=0; y<N_PIXELS; y++)
			{printf ("%+4.4f     ", output_ground_model_x[x][y]);}
			 printf ("\n");
		}

		printf ("----------\n");
		for (int x=0; x<N_PIXELS; x++)
		{
			for (int y=0; y<N_PIXELS; y++)
			{printf ("%+4.4f     ", output_ground_model_y[x][y]);}
			 printf ("\n");
		}
	}



};

#endif