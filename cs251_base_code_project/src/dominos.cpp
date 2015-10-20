/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */


#include "cs251_base.hpp"
#include "render.hpp"
#include <cstdlib>
#include <cmath>

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
 


  dominos_t::dominos_t()
  {
    //! **************************************************************************************************************************************\n

    //! Air Particles in the Piston ------------------------------------------------------------------------------------------------------------------\n
    //! These are the air particles which are used in inlet and outlet of the combustion engine \n
 //! A total of 84 air particles are constructed in different position in the combustion engine to feed its inlet \n
    
    
    {
     

      b2CircleShape circle;
      circle.m_radius = 0.08;	/**< circular air particles of radius 0.08 */
          //new
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.05f;	/**< light air particles of density 0.05 */
      ballfd.friction = 0.0f; /**< friction of air particles is 0 */
      ballfd.restitution = 0.75f;
      //ballfd.isBullet = false;
      ballfd.filter.groupIndex = 1; /**< group index of air partcles is 1 so that it do not collide with the arm of piston */

      for (int i = 0; i <5 ; ++i)
  {
    for(int j=0;j<4;j++)
    {
      //float r1=rand() % 3 ;
      //float r2=rand() % 2 ;
    b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(-6.0f + i*2, 22.0f+j*0.5);
    spherebody = m_world->CreateBody(&ballbd);
    spherebody->CreateFixture(&ballfd);
    spherebody->SetUserData(this);
    spherebody->SetLinearVelocity(b2Vec2(0,-0.5));
      spherebody->SetGravityScale(0.1);
  }
  }
    }

    {
     

      b2CircleShape circle;
      circle.m_radius = 0.08;
          //new
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.5f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.95f;
      //ballfd.isBullet = false;
      ballfd.filter.groupIndex = 1;

      for (int i = 0; i <8 ; ++i)
  {
    for(int j=0;j<8;j++)
    {
      //float r1=rand() % 3 ;
      //float r2=rand() % 2 ;
    b2BodyDef ballbd;
    ballbd.bullet=false;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(9.f + i*0.5, 29.5f+j*0.5);
    spherebody1 = m_world->CreateBody(&ballbd);
    spherebody1->CreateFixture(&ballfd);
    spherebody1->SetUserData(this);
    spherebody1->SetLinearVelocity(b2Vec2(0,-0.5));
      spherebody1->SetGravityScale(0.1);
  }
  }
    }

    //The main piston
//! **************************************************************************************************************************************\n

	//! The Main Piston ------------------------------------------------------------------------------------------------------------------\n
//! The main piston consist of three major components.\n
	//! They are as follows.\n
    {
	//The upper piston
//! The Upper Piston -----------------------------------------------------------------------------------------------------------------\n

//! The upper part of piston is a rectangular block which perform to and fro motion in the vertical deirection based on the motion of revolving part of the piston described below.\n

      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(9.1f, 3.25f);  /**< rectangular block of size 9.1 * 3.25 */

      
	b2BodyDef bd;
	bd.type=b2_dynamicBody;		/**< rcetangular block is a dynamic body */
	bd.position.Set(2.0f, 18.0f); /**< the centre is fixed at (2.0,18.0) */
	b2 = m_world->CreateBody(&bd);
  b2FixtureDef *ffd=new b2FixtureDef;
  
  ffd->shape = new b2PolygonShape;
  ffd->shape=&shape;
  ffd->restitution=0.5f; /**< coefficient of restitution is 0.5 */
  ffd->density=2.f; 	/**< density of rectangular block is 2 */

	b2->CreateFixture(ffd);



      }

	//The lower revolving body
      //change

//! The Lower Revolving Body ------------------------------------------------------------------------------------------------------------------\n

//! this lower rotaing part of piston is attached to the upper rectangular part of the piston by distance joint such that the distance between a vertex of the this rotating part and the other rectangular block is fixed.\n
      
      {
	b2PolygonShape shape;
	shape.SetAsBox(3.0f, 3.0f); /**< square of side 3 */

	b2BodyDef bd;
	bd.type = b2_kinematicBody; /**< defined as a kinematic body */
	bd.position.Set(2.0f, 1.0f); /**< position of centre of this squar efixed at (2.0,1.0) */
	bpiston = m_world->CreateBody(&bd);
	bpiston->CreateFixture(&shape, 10.0f);
	
	bpiston->SetAngularVelocity( 0-w2 );/**< angular velocity of this square is a variable w2 which can be changed by the user with '+' and '-' key on the keyboard */
      }

//! The Main Gear ------------------------------------------------------------------------------------------------------------------\n

//! The main gear is attached to the lower rotating ortion of the piston which in turn also roates the main gear.\n
//! The main gear is basically a circle of radius 4 cm.\n
//! The spikes on gear is in form of triangles.\n
//! The layer of triangle has a height of 1.5\n
//! Their are total 24 triangle shaped attache don the circle to make gear.\n
 
      {
        float x0 = 0.0f, y0 = 0.0, r= 4.5; /**< radius of main gear if 4.5 */
        double pi=3.14159265359;
        
        b2EdgeShape shape; 
        b2BodyDef bd;
        bd.position.Set(2.0f, 1.0f); /**< centre of gear set at (2.0,1.0)*/
        bd.type=b2_kinematicBody; /**< gear is a kinematic body which rotates with the same angular velocity as the lower rotating square. */
        bpiston2 = m_world->CreateBody(&bd);
        //b1->GetLocalCenter();
        float x=x0 + r ;
        float y=y0 ;
        for(int i=0; i <48; i++){
          if(i%2==0){
            int j=i/2;
          float tempx = x0 +(r+1.5) * cos((15*j+7.5)*pi/180);
          float tempy = y0 + (r+1.5) * sin((15*j+7.5)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          bpiston2->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;}
          else
          {
            int j=(i+1)/2;
            float tempx = x0 + r * cos((15*j)*pi/180);
          float tempy = y0 + r * sin((15*j)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          bpiston2->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;
          }
        }
        bpiston2->SetAngularVelocity(0-w2);/**< angular velocity of the main gear is same as the angular velocity of rotating part i.e. w2*/
    }

	//Line joining upper and lower parts of piston
	b2DistanceJointDef jointDef;/**< distancejoint between the upper part of the piston and the lower part*/

	b2Vec2 anchorA;
	b2Vec2 anchorB;
 

	anchorA.Set(2.0f,18.0f); /**< anchorA of distance joint set at (2.0,18.0)*/
	anchorB.Set(-1.0f,4.0f); /**< anchorB of distance joint set at (-1.0,4.0)*/
  

	jointDef.Initialize(b2, bpiston, anchorA, anchorB);
  

	jointDef.collideConnected = true;
  


      //b2RevoluteJointDef jd;
     // b2Vec2 anchor;
     // anchor.Set(-28.0f, 35.5f);
      //jd.Initialize(b2, b4,anchor);
      m_world->CreateJoint(&jointDef);

    }
     
    //four circles stationary at above
//The Four Stationary Circles
   	//! **************************************************************************************************************************************\n

	//! The Four Stationary Circles ---------------------------------------------------------------------------------------------------\n
//! A total of four static circles are ther at above which put constraints in the motion of left and right inlet and outlet piston\n
  	{
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 0.7f; /**< radius of all four cicles is 0.7*/

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_staticBody; /**< all four circles are static body*/
      ballbd.position.Set(-11.0f, 38.0f);/**< position of first circle at (-11.0,38.0)*/
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 0.7f;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_staticBody;
      ballbd.position.Set(15.0f, 38.0f);/**< the position of second circle at (15.0,38.0)*/
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }/////////////////////////////////////////////!
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 0.7f;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_staticBody;
      ballbd.position.Set(-2.0f, 45.0f);/**< the position of third circle at (-2.0,45.0)*/
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 0.7f;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_staticBody;
      ballbd.position.Set(6.0f, 45.0f);/**< the position of forth circle at (6.0,45.0)*/
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    
//valve system left side
//! **************************************************************************************************************************************\n
	//! The Left Side System Valve ---------------------------------------------------------------------------------------------------\n

//! The left side system valve consist of three parts.\n
//! All the three part of the valve are jointe dtogether by distance and revoliute joint to perform in an unit\n
    {
	//Left Valve part 1

//! The Left Valve Part I ---------------------------------------------------------------------------------------------------\n

//! This is the lowest part of the valve which opens and closes at regular interval\n
//! This a unregular hexagon shaped object.\n
      b2Body* bp1;
	{
      	b2PolygonShape poly7;
      	b2BodyDef bd;
      	b2Vec2 vertices[6];
      	vertices[0].Set(-5.0f,24.5f);		//done
      	vertices[1].Set(-1.0f,28.0f);		//done
      	vertices[2].Set(-1.9f,29.2f);		//done
      	vertices[3].Set(-4.2f,28.7f);
      	vertices[4].Set(-5.0f,28.0f);
      	vertices[5].Set(-5.7f,25.5f);		
      	
      	poly7.Set(vertices, 6); /**< polygon of 6 sides*/
      	bd.type=b2_dynamicBody;	/**< defined as dynamic body*/

      	bp1 = m_world->CreateBody(&bd);
      	b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f; /**< density of this part of valve is 1*/

        fd->restitution=1.f; /**< coefficient of restitution of this part of the valve is 1*/
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly7;

      	bp1->CreateFixture(fd);
        bp1->SetGravityScale(0);

    }

	//Left Valve part 2

//! The Left Valve part II ---------------------------------------------------------------------------------------------------\n

//! This is the arm of the valve which pushes and pulls back the part 1 up and down.\n
//! This part is a polygon of 4 sides.\n
      b2Body* bp2;
      {
		b2PolygonShape poly8;
		b2BodyDef bd;
		b2Vec2 vertices[4];

		vertices[0].Set(-5.0f,28.0f);
		vertices[1].Set(-4.2f,28.7f);
		vertices[2].Set(-10.22f,35.8f);
		vertices[3].Set(-11.35f,35.f);

		poly8.Set(vertices,4);
		bd.type = b2_dynamicBody;/**< defined as a dynamic body*/

		bp2 = m_world->CreateBody(&bd);
		b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f; /**< density of this part of valve is 1.0*/
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly8;
        fd->restitution=1.f; /**< coefficient of restitution is 1.0*/
        fd->isSensor=true;
        fd->filter.groupIndex = -1; /** group index of this part 1s 1 so that air particals can move freely in the outlet.*/
		bp2->CreateFixture(fd);
	    bp2->SetGravityScale(0);
      }

	//Left Valve part 3

      //! The Left Side valve part III ---------------------------------------------------------------------------------------------------\n

//! this is rectangular object which rotate about its centre when force is applied by the upper rotating triangle \n
      b2PolygonShape shape;
      shape.SetAsBox(6.3f, 0.4f); /**< rectangle of size 6.3 * 0.4 */

      b2BodyDef bd;
      bd.position.Set(-6.0f, 38.75f); /**< centre of rectangle  set at (-6.0,38.75) */
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 100.f; /**< density of rectangle set as 100*/

      fd->restitution=0.0f; /**< restitution of rectangle set as 0*/
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.0f, 0.0f);
      b2BodyDef bd2;
      bd2.position.Set(-7.0f, 38.75f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(-2 ,0); /**< centre of rotation locally set at (-2,0) */ 
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = true;

      body->SetTransform( body->GetPosition(), 0.6 );

      m_world->CreateJoint(&jointDef);
      
    

	//Joints of Valve Left     
	b2DistanceJointDef jointDef1;
	b2DistanceJointDef jointDef2;
  	b2DistanceJointDef jointDef3;
	//b2DistanceJointDef jointDef4;

	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 anchorC;
	b2Vec2 anchorD;
	b2Vec2 anchor5;
	b2Vec2 anchor6;
	//b2Vec2 anchor7;
	//b2Vec2 anchor8;

	anchorA.Set(-4.2f,28.7f);
	anchorB.Set(-4.2f,28.7f);
	anchorC.Set(-5.0f,28.0f);
	anchorD.Set(-5.0f,28.0f);
	anchor5.Set(-11.35f,35.f);
	anchor6.Set(-11.35f,35.f);
	//anchor7.Set(-38.22f,35.8f);
	//anchor8.Set(-38.22f,35.8f);
  	
	jointDef1.Initialize(bp1, bp2, anchorA, anchorB);
	jointDef2.Initialize(bp1, bp2, anchorC, anchorD);
	jointDef3.Initialize(bp2,body,anchor5,anchor6);
	//jointDef4.Initialize(bp2,body,anchor7,anchor8);
  	
	jointDef1.collideConnected = true;
	jointDef2.collideConnected = true;
	jointDef3.collideConnected = true;
	//jointDef4.collideConnected = true;
  	
     //b2RevoluteJointDef jd;
     // b2Vec2 anchor;
     // anchor.Set(-28.0f, 35.5f);
      //jd.Initialize(b2, b4,anchor);
    m_world->CreateJoint(&jointDef1);
    m_world->CreateJoint(&jointDef2);
    m_world->CreateJoint(&jointDef3);
    //m_world->CreateJoint(&jointDef4);
      
    }
    
    
//////////////////////////////outlet
//! **************************************************************************************************************************************\n

    //! Outlet of the system ------------------------------------------------------------------------------------------------------------------\n
//! The outlet is made up of three edge shaped objects./n
//! The group index of these three edges are -1 so that air particles collide with them.\n
    
    {
        float x0 = -15.0f, y0 = 33.f;/**< first edge is from (-15.0,33.0) to (-14.0,37.0)*/

         
        b2EdgeShape shape; 
        b2BodyDef bd;
        lineleft = m_world->CreateBody(&bd);
  
          shape.Set(b2Vec2(x0,y0), b2Vec2(-14, 37));
          b2FixtureDef fd;
          fd.shape = &shape;
          fd.filter.groupIndex = -1;
          lineleft->CreateFixture(&fd); 
          lineleft->SetUserData(this);

    }

    {
        float x0 = -8.0f, y0 = 28.5;/**< second edge is from (-8.0,28.5) to (-15.0,33.0)*/

         
        b2EdgeShape shape; 
        b2BodyDef bd;
        lineleft1 = m_world->CreateBody(&bd);
  
          shape.Set(b2Vec2(x0,y0), b2Vec2(-15, 33));
          b2FixtureDef fd;
          fd.shape = &shape;
          fd.filter.groupIndex = -1;
          lineleft1->CreateFixture(&fd); 
          lineleft1->SetUserData(this);

    }

    {
        float x0 = -5.0f, y0 = 33.f;/**< third edge is from (-5.0,33.0) to (-14.0,37.0)*/

         
        b2EdgeShape shape; 
        b2BodyDef bd;
        lineleft2 = m_world->CreateBody(&bd);
  
          shape.Set(b2Vec2(x0,y0), b2Vec2(-14, 37));
          b2FixtureDef fd;
          fd.shape = &shape;
          fd.filter.groupIndex = -1;
          lineleft2->CreateFixture(&fd); 
          lineleft2->SetUserData(this);

    }
////////////endoutlet

    //////////////////inlet
//! **************************************************************************************************************************************\n

    //! Inlet of the system ------------------------------------------------------------------------------------------------------------------\n
//! The inlet is made up of three edge shaped objects./n
//! The group index of these three edges are -1 so that air particles collide with them.\n


    
    {
        float x0 = 22.0f, y0 = 35;/**< first edge is from(22.0,35.0) to (19.0,38.0)*/

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
       
          shape.Set(b2Vec2(x0,y0), b2Vec2(19, 38));
          b2FixtureDef fd;
          fd.shape = &shape;
          fd.filter.groupIndex = -1;
          fd.restitution=1.0f;
          b1->CreateFixture(&fd); 

    }

    {
        float x0 = 12.0f, y0 = 28.5;/**< second edge is from (12.0,28.5) to (22.0,35.0)*/

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
       
          shape.Set(b2Vec2(x0,y0), b2Vec2(22, 35));
          b2FixtureDef fd;
          fd.shape = &shape;
          fd.filter.groupIndex = -1;
          fd.restitution=1.0f;
          b1->CreateFixture(&fd); 

    }

    {
        float x0 = 9.0f, y0 = 33;/**< third edge is from (9.0,33.0) to (19.0,38.0)*/

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
       
          shape.Set(b2Vec2(x0,y0), b2Vec2(19, 38));
          b2FixtureDef fd;
          fd.shape = &shape;
          fd.filter.groupIndex = -1;
          fd.restitution=1.0f;
          b1->CreateFixture(&fd); 

    }

    ////////////////////////endinlet
///////////////////////////////////////////////




//valve system right side
//! **************************************************************************************************************************************\n

	//! The Right Side System Valve ---------------------------------------------------------------------------------------------------\n
//! The right side system valve consist of three parts.\n
//! All the three part of the valve are jointe dtogether by distance and revoliute joint to perform in an unit\n

    {
	//right Valve part 1

//! The Right Valve Part I ---------------------------------------------------------------------------------------------------\n

//! This is the lowest part of the valve which opens and closes at regular interval\n
//! This a unregular hexagon shaped object.\n
      b2Body* rva;
	{
      	b2PolygonShape poly7;
      	b2BodyDef bd;
      	b2Vec2 vertices[6];
      	vertices[0].Set(9.0f,24.5f);		//done
      	vertices[1].Set(5.0f,28.0f);		//done
      	vertices[2].Set(5.9f,29.2f);		//done
      	vertices[3].Set(8.2f,28.7f);
      	vertices[4].Set(9.f,28.0f);
      	vertices[5].Set(9.7f,25.5f);		
      	
      	poly7.Set(vertices, 6);
      	bd.type=b2_dynamicBody;  /**< polygon shaped dynamic object of 6 sides*/

      	rva = m_world->CreateBody(&bd);
      	b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f; /**< density of this part is 1*/
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly7;
      	rva->CreateFixture(fd);
        rva->SetGravityScale(0);

    }

	//right Valve part 2

//! The Right Valve part II ---------------------------------------------------------------------------------------------------\n

//! This is the arm of the valve which pushes and pulls back the part 1 up and down.\n
//! This part is a polygon of 4 sides.\n
      b2Body* rvb;
      {
		b2PolygonShape poly8;
		b2BodyDef bd;
		b2Vec2 vertices[4];

		vertices[0].Set(9.0f,28.0f);
		vertices[1].Set(8.2f,28.7f);
		vertices[2].Set(14.22f,35.8f);
		vertices[3].Set(15.35f,35.f);

		poly8.Set(vertices,4);
		bd.type = b2_dynamicBody;/**< defined as a dynamic body*/

		rvb = m_world->CreateBody(&bd);
		b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f;/**< density of this part is 1*/
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly8;
        fd->restitution=1.f;
        fd->filter.groupIndex=-1;/** group index is -1 so that air particles do not collide with the arm of valve and move freely in the outlet*/
        fd->isSensor=true;
		rvb->CreateFixture(fd);
	    rvb->SetGravityScale(0);
      }

	//right Valve part 3
      
//! The Right Side valve part III ---------------------------------------------------------------------------------------------------\n

//! this is rectangular object which rotate about its centre when force is applied by the upper rotating triangle \n
      b2PolygonShape shape;
      shape.SetAsBox(6.3f, 0.4f);/**< rectabgle shaped object of size 6.3 * 0.4*/

      b2BodyDef bd;
      bd.position.Set(10.0f, 38.75f);/**< centre of rectangle fixed at (10.0,38.75)*/
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 100.f;/**< density of rectangle 100*/
      fd->restitution=0.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.0f, 0.0f);
      b2BodyDef bd2;
      bd2.position.Set(11.0f, 38.75f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(2 ,0);/**< centre of roatation of the rectangle is local coordinate (2,0)*/
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = true;

      body->SetTransform( body->GetPosition(), -0.6 );

      m_world->CreateJoint(&jointDef);
      
    

	//Joints of Valve Left     
	b2DistanceJointDef jointDef1;
	b2DistanceJointDef jointDef2;
  	b2DistanceJointDef jointDef3;
	//b2DistanceJointDef jointDef4;

	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 anchorC;
	b2Vec2 anchorD;
	b2Vec2 anchor5;
	b2Vec2 anchor6;
	//b2Vec2 anchor7;
	//b2Vec2 anchor8;

	anchorA.Set(8.2f,28.7f);
	anchorB.Set(8.2f,28.7f);
	anchorC.Set(9.f,28.0f);
	anchorD.Set(9.f,28.0f);
	anchor5.Set(15.35f,35.f);
	anchor6.Set(15.35f,35.f);
	//anchor7.Set(-38.22f,35.8f);
	//anchor8.Set(-38.22f,35.8f);
  	
	jointDef1.Initialize(rva, rvb, anchorA, anchorB);
	jointDef2.Initialize(rva, rvb, anchorC, anchorD);
	jointDef3.Initialize(rvb,body,anchor5,anchor6);
	//jointDef4.Initialize(bp2,body,anchor7,anchor8);
  	
	jointDef1.collideConnected = true;
	jointDef2.collideConnected = true;
	jointDef3.collideConnected = true;
	//jointDef4.collideConnected = true;
  	
     //b2RevoluteJointDef jd;
     // b2Vec2 anchor;
     // anchor.Set(-28.0f, 35.5f);
      //jd.Initialize(b2, b4,anchor);
    m_world->CreateJoint(&jointDef1);
    m_world->CreateJoint(&jointDef2);
    m_world->CreateJoint(&jointDef3);
    //m_world->CreateJoint(&jointDef4);
      
    }
    
	

//both side stand
//! **************************************************************************************************************************************\n

	//! The Both Side Stand ---------------------------------------------------------------------------------------------------\n
//! There are two rigid stand one each in the left and right side of the combustion engine.\n

	//Left stand
    {
      	b2PolygonShape poly;
      	b2BodyDef bd;
      	b2Vec2 vertices[4];/**< left side stand is a static body of 4 sides.*/
      	vertices[0].Set(-8.0f,7.5f);
      	vertices[1].Set(-6.95f,7.5f);
      	vertices[2].Set(-6.95f,27.0f);
      	vertices[3].Set(-8.0f,28.5f);
      	poly.Set(vertices, 4);
      	bd.type=b2_staticBody;

      	b2Body* body = m_world->CreateBody(&bd);
      	 b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
      fd->restitution=1.f;
      body->CreateFixture(fd);
      	

    }
	//Right stand
    {
      	b2PolygonShape poly1; /**< right side stand is a static body of 4 sides.*/
      	b2BodyDef bd1;
      	b2Vec2 vertices[4];
      	vertices[0].Set(12.0f,7.5f);
      	vertices[1].Set(10.95f,7.5f);
      	vertices[2].Set(10.95f,27.0f);
      	vertices[3].Set(12.f,28.5f);
      	poly1.Set(vertices, 4);
      	bd1.type=b2_staticBody;

      	b2Body* body = m_world->CreateBody(&bd1);
      	 b2FixtureDef *fd1 = new b2FixtureDef;
         body->SetUserData(NULL);
      fd1->density = 1.f;
      fd1->shape = new b2PolygonShape;
      fd1->restitution=1.f;
      fd1->shape = &poly1;
      body->CreateFixture(fd1);
      	
      	
    }
 

	//Rotating triangle
//! **************************************************************************************************************************************\n

	//! The Upper Rotating Triangle ---------------------------------------------------------------------------------------------------\n
//! The rotating triangle provides motion  to the inlet and outlet valve of the system.\n
//! It a dynamic body made up of three rectangles foeming an isoceles triangle.\n

    {
      double pi=3.14159265359;
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_kinematicBody;
      bd->position.Set(2.0f,39.1547f);
      bd->angle=150*pi/180;
      bd->fixedRotation = true;
       //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.0;
      fd1->friction = 1.0;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;

      b2PolygonShape bs1;
      bs1.SetAsBox(4.2,0.2, b2Vec2(0.0f,-2.1f), 0);



      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.0;
      fd2->friction = 0.0;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2.7, b2Vec2(2.6f,0.f), 0.643501109f);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 1.0;
      fd3->friction = 0.0;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2.7, b2Vec2(-2.6f,0.f), -0.643501109f);
      fd3->shape = &bs3;


      box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
      box1->SetGravityScale(0);
      box1->SetAngularVelocity(w2/2);

    }


    //just uppar stand
    {
    	b2PolygonShape poly1;
      	b2BodyDef bd1;
      	b2Vec2 vertices[4];
      	vertices[0].Set(-3.0f,29.0f);
      	vertices[1].Set(7.0f,29.0f);
      	vertices[2].Set(9.0f,33.0f);
      	vertices[3].Set(-5.0f,33.0f);
      	poly1.Set(vertices, 4);
      	bd1.type=b2_staticBody;

      	b2Body* body = m_world->CreateBody(&bd1);
      	 b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.f;
      fd1->shape = new b2PolygonShape;
      fd1->shape = &poly1;
      fd1->restitution=1.f;
      body->CreateFixture(fd1);
      	
    }

    //gear1
//! **************************************************************************************************************************************\n

	//! Gears ---------------------------------------------------------------------------------------------------\n
//! There are a total of 5 gears below the main gear which when connected to main gear produces different speed angular speed based on the size of gear.\n
//! \n

      {
        float x0 = 0.0f, y0 = 0.0, r= 5.5;/**< gear 1 radius is 5.5*/
        double pi=3.14159265359;
        
        b2EdgeShape shape; 
        b2BodyDef bd;
        bd.fixedRotation=true;
        bd.position.Set(pos+14, -10.8f);
        bd.type=b2_kinematicBody;
        b2FixtureDef fd;
        fd.isSensor=false;
        //fd.filter.groupIndex = 1;

        gear1 = m_world->CreateBody(&bd);
        //b1->GetLocalCenter();
        float x=x0 + r * sin(6*pi/180) ;
        float y=y0 + r * cos(6*pi/180) ;
        for(int i=0; i <60; i++){ /**< total number of teeth in gear 1 is 30*/
          if(i%2==0){
            int j=i/2;
          float tempx = x0 + (r+1.5) * sin((12*j+12)*pi/180);
          float tempy = y0 + (r+1.5) * cos((12*j+12)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear1->CreateFixture(&fd); 
          x=tempx;
          y=tempy;}
          else
          {
            int j=(i+1)/2;
            float tempx = x0 +r * sin((12*j+6.0)*pi/180);
          float tempy = y0 + r * cos((12*j+6.0)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear1->CreateFixture(&fd); 
          x=tempx;
          y=tempy;
          }
        }
        gear1->SetGravityScale(0);
        gear1->SetAngularVelocity(w1*4.5/5.5);

    }
//endgear1
//!\n
    //gear2
    {
        float x0 = 0.0f, y0 = 0.0, r= 4.5; /**< radius of gear 2 is 4.5*/
        double pi=3.14159265359;
        
        b2EdgeShape shape; 
        b2BodyDef bd;
        bd.fixedRotation=true;
        bd.position.Set(pos, -9.8f);
        bd.type=b2_kinematicBody;
        b2FixtureDef fd;
        fd.isSensor=false;
        //fd.filter.groupIndex = 1;

        gear2 = m_world->CreateBody(&bd);
        //b1->GetLocalCenter();
        float x=x0 + r * sin(7.5*pi/180) ;
        float y=y0 + r * cos(7.5*pi/180) ;
        for(int i=0; i <48; i++){/**< total number of teeth in gear 2 is 24*/
          if(i%2==0){
            int j=i/2;
          float tempx = x0 + (r+1.5) * sin((15*j+15)*pi/180);
          float tempy = y0 + (r+1.5) * cos((15*j+15)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear2->CreateFixture(&fd); 
          x=tempx;
          y=tempy;}
          else
          {
            int j=(i+1)/2;
            float tempx = x0 +r * sin((15*j+7.5)*pi/180);
          float tempy = y0 + r * cos((15*j+7.5)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear2->CreateFixture(&fd); 
          x=tempx;
          y=tempy;
          }
        }
        gear2->SetGravityScale(0);
        gear2->SetAngularVelocity(w2);

    }
//endgear2
//! \n
    //gear3
    {
        float x0 = 0.0f, y0 = 0.0, r= 3.75; /**< radius of gear 3 is 3.75*/
        double pi=3.14159265359;
        
        b2EdgeShape shape; 
        b2BodyDef bd;
        bd.fixedRotation=true;
        bd.position.Set(pos-14, -9.05f);
        bd.type=b2_kinematicBody;
        b2FixtureDef fd;
        fd.isSensor=false;
        //fd.filter.groupIndex = 1;

        gear3 = m_world->CreateBody(&bd);
        //b1->GetLocalCenter();
        float x=x0 + r * sin(9*pi/180) ;
        float y=y0 + r * cos(9*pi/180) ;
        for(int i=0; i <40; i++){/**< total number of teeth in gear 3 is 20*/
          if(i%2==0){
            int j=i/2;
          float tempx = x0 + (r+1.5) * sin((18*j+18)*pi/180);
          float tempy = y0 + (r+1.5) * cos((18*j+18)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear3->CreateFixture(&fd); 
          x=tempx;
          y=tempy;}
          else
          {
            int j=(i+1)/2;
            float tempx = x0 +r * sin((18*j+9)*pi/180);
          float tempy = y0 + r * cos((18*j+9)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear3->CreateFixture(&fd); 
          x=tempx;
          y=tempy;
          }
        }
        gear3->SetGravityScale(0);
        gear3->SetAngularVelocity(w3*4.5/3.75);

    }
//endgear3
//! \n
    //gear4
    {
        float x0 = 0.0f, y0 = 0.0, r= 3;/**< radius of gear 4 is 3*/
        double pi=3.14159265359;
        
        b2EdgeShape shape; 
        b2BodyDef bd;
        bd.fixedRotation=true;
        bd.position.Set(pos-28, -8.3f);
        bd.type=b2_kinematicBody;
        b2FixtureDef fd;
        fd.isSensor=false;
        //fd.filter.groupIndex = 1;

        gear4 = m_world->CreateBody(&bd);
        //b1->GetLocalCenter();
        float x=x0 + r * sin(11.25*pi/180) ;
        float y=y0 + r * cos(11.25*pi/180) ;
        for(int i=0; i <32; i++){/** < total number of teeth in gear 4 is 16*/
          if(i%2==0){
            int j=i/2;
          float tempx = x0 + (r+1.5) * sin((22.5*j+22.5)*pi/180);
          float tempy = y0 + (r+1.5) * cos((22.5*j+22.5)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear4->CreateFixture(&fd); 
          x=tempx;
          y=tempy;}
          else
          {
            int j=(i+1)/2;
            float tempx = x0 +r * sin((22.5*j+11.25)*pi/180);
          float tempy = y0 + r * cos((22.5*j+11.25)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear4->CreateFixture(&fd); 
          x=tempx;
          y=tempy;
          }
        }
        gear4->SetGravityScale(0);
        gear4->SetAngularVelocity(w4*4.5/3);

    }
//endgear4
//! \n
    //gear5
    {
        float x0 = 0.0f, y0 = 0.0, r= 2.25;/**< radius of gear 5 is 2.25*/
        double pi=3.14159265359;
        
        b2EdgeShape shape; 
        b2BodyDef bd;
        bd.fixedRotation=true;
        bd.position.Set(pos-42, -7.55f);
        bd.type=b2_kinematicBody;
        b2FixtureDef fd;
        fd.isSensor=false;
        //fd.filter.groupIndex = 1;

        gear5 = m_world->CreateBody(&bd);
        //b1->GetLocalCenter();
        float x=x0 + r * sin(15*pi/180) ;
        float y=y0 + r * cos(15*pi/180) ;
        for(int i=0; i <24; i++){/**< total number of teeth in gear 5 is 12*/
          if(i%2==0){
            int j=i/2;
          float tempx = x0 + (r+1.5) * sin((30*j+30)*pi/180);
          float tempy = y0 + (r+1.5) * cos((30*j+30)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear5->CreateFixture(&fd); 
          x=tempx;
          y=tempy;}
          else
          {
            int j=(i+1)/2;
            float tempx = x0 +r * sin((30*j+15)*pi/180);
          float tempy = y0 + r * cos((30*j+15)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear5->CreateFixture(&fd); 
          x=tempx;
          y=tempy;
          }
        }
        gear5->SetGravityScale(0);
        gear5->SetAngularVelocity(w5*4.5/2.25);

    }
//endgear5
/*
    ///reversegear
    {
        float x0 = 0.0f, y0 = 0.0, r= 4.5;
        double pi=3.14159265359;
        
        b2EdgeShape shape; 
        b2BodyDef bd;
        bd.fixedRotation=true;
        bd.position.Set(-17.2f, 1.0f);
        bd.type=b2_kinematicBody;
        b2FixtureDef fd;
        fd.isSensor=false;
        //fd.filter.groupIndex = 1;

        gear1 = m_world->CreateBody(&bd);
        //b1->GetLocalCenter();
        float x=x0 + r * cos(7.5*pi/180) ;
        float y=y0 + r * sin(7.5*pi/180) ;
        for(int i=0; i <48; i++){
          if(i%2==0){
            int j=i/2;
          float tempx = x0 + (r+1.5) * cos((15*j+15)*pi/180);
          float tempy = y0 + (r+1.5) * sin((15*j+15)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear1->CreateFixture(&fd); 
          x=tempx;
          y=tempy;}
          else
          {
            int j=(i+1)/2;
            float tempx = x0 +r * cos((15*j+7.5)*pi/180);
          float tempy = y0 + r * sin((15*j+7.5)*pi/180);
          shape.Set(b2Vec2(x,y), b2Vec2(tempx, tempy));
          fd.shape = &shape;
          gear1->CreateFixture(&fd); 
          x=tempx;
          y=tempy;
          }
        }
        gear1->SetGravityScale(0);
        gear1->SetAngularVelocity(2);

    }
///endreversegear

      */




  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
