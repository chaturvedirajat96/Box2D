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
  /**  The is the constructor
   * This is the documentation block for the constructor.
   */


  dominos_t::dominos_t()
  {
    //Ground
    /*! \var b1
     * \brief pointer to the body ground
     */
     

    

    //The revolving horizontal platform
  
    //random air particles
    
//The train of small spheres
    
    {
     

      b2CircleShape circle;
      circle.m_radius = 0.08;
          //new
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.05f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.75f;
      //ballfd.isBullet = false;
      ballfd.filter.groupIndex = 1;

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
    {
	//The upper piston
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(9.1f, 3.25f);

      
	b2BodyDef bd;
	bd.type=b2_dynamicBody;
	bd.position.Set(2.0f, 18.0f);
	b2 = m_world->CreateBody(&bd);
  b2FixtureDef *ffd=new b2FixtureDef;
  
  ffd->shape = new b2PolygonShape;
  ffd->shape=&shape;
  ffd->restitution=0.5f;
  ffd->density=2.f;

	b2->CreateFixture(ffd);



      }

	//The lower revolving body
      //change
      
      {
	b2PolygonShape shape;
	shape.SetAsBox(3.0f, 3.0f);

	b2BodyDef bd;
	bd.type = b2_kinematicBody;
	bd.position.Set(2.0f, 1.0f);
	bpiston = m_world->CreateBody(&bd);
	bpiston->CreateFixture(&shape, 10.0f);
	
	bpiston->SetAngularVelocity( 0-w2 );
      }

 
      {
        float x0 = 0.0f, y0 = 0.0, r= 4.5;
        double pi=3.14159265359;
        
        b2EdgeShape shape; 
        b2BodyDef bd;
        bd.position.Set(2.0f, 1.0f);
        bd.type=b2_kinematicBody;
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
        bpiston2->SetAngularVelocity(0-w2);
    }

	//Line joining upper and lower parts of piston
	b2DistanceJointDef jointDef;

	b2Vec2 anchorA;
	b2Vec2 anchorB;
 

	anchorA.Set(2.0f,18.0f);
	anchorB.Set(-1.0f,4.0f);
  

	jointDef.Initialize(b2, bpiston, anchorA, anchorB);
  

	jointDef.collideConnected = true;
  


      //b2RevoluteJointDef jd;
     // b2Vec2 anchor;
     // anchor.Set(-28.0f, 35.5f);
      //jd.Initialize(b2, b4,anchor);
      m_world->CreateJoint(&jointDef);

    }
     
    //four circles stationary at above
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
      ballbd.position.Set(-11.0f, 38.0f);
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
      ballbd.position.Set(15.0f, 38.0f);
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
      ballbd.position.Set(-2.0f, 45.0f);
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
      ballbd.position.Set(6.0f, 45.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    
//valve system left side
    {
	//Left Valve part 1
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
      	
      	poly7.Set(vertices, 6);
      	bd.type=b2_dynamicBody;

      	bp1 = m_world->CreateBody(&bd);
      	b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f;
        fd->restitution=1.f;
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly7;

      	bp1->CreateFixture(fd);
        bp1->SetGravityScale(0);

    }

	//Left Valve part 2
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
		bd.type = b2_dynamicBody;

		bp2 = m_world->CreateBody(&bd);
		b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f;
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly8;
        fd->restitution=1.f;
        fd->isSensor=true;
        fd->filter.groupIndex = -1;
		bp2->CreateFixture(fd);
	    bp2->SetGravityScale(0);
      }

	//Left Valve part 3
      
      b2PolygonShape shape;
      shape.SetAsBox(6.3f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(-6.0f, 38.75f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 100.f;

      fd->restitution=0.0f;
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
      jointDef.localAnchorA.Set(-2 ,0);
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
    
    {
        float x0 = -15.0f, y0 = 33.f;

         
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
        float x0 = -8.0f, y0 = 28.5;

         
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
        float x0 = -5.0f, y0 = 33.f;

         
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
    {
        float x0 = 22.0f, y0 = 35;

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
        float x0 = 12.0f, y0 = 28.5;

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
        float x0 = 9.0f, y0 = 33;

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
    {
	//right Valve part 1
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
      	bd.type=b2_dynamicBody;

      	rva = m_world->CreateBody(&bd);
      	b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f;
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly7;
      	rva->CreateFixture(fd);
        rva->SetGravityScale(0);

    }

	//right Valve part 2
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
		bd.type = b2_dynamicBody;

		rvb = m_world->CreateBody(&bd);
		b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f;
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly8;
        fd->restitution=1.f;
        fd->filter.groupIndex=-1;
        fd->isSensor=true;
		rvb->CreateFixture(fd);
	    rvb->SetGravityScale(0);
      }

	//right Valve part 3
      
      b2PolygonShape shape;
      shape.SetAsBox(6.3f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(10.0f, 38.75f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 100.f;
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
      jointDef.localAnchorA.Set(2 ,0);
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
	//Left stand
    {
      	b2PolygonShape poly;
      	b2BodyDef bd;
      	b2Vec2 vertices[4];
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
      	b2PolygonShape poly1;
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

    ///////////gear1


      {
        float x0 = 0.0f, y0 = 0.0, r= 5.5;
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
        for(int i=0; i <60; i++){
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
///endgear1

    ///gear2
    {
        float x0 = 0.0f, y0 = 0.0, r= 4.5;
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
        for(int i=0; i <48; i++){
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
///endgear2

    ///gear3
    {
        float x0 = 0.0f, y0 = 0.0, r= 3.75;
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
        for(int i=0; i <40; i++){
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
///endgear3

    ///gear4
    {
        float x0 = 0.0f, y0 = 0.0, r= 3;
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
        for(int i=0; i <32; i++){
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
///endgear4

    ///gear5
    {
        float x0 = 0.0f, y0 = 0.0, r= 2.25;
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
        for(int i=0; i <24; i++){
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
///endgear5
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
