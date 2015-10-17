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
     /*
    b2Body* b1;
    {

      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    */
/*
    //Top horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    */
/*
    //Dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;

      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
    */
/*
    //Another horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);

      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
*/
/*
    //The pendulum that knocks the dominos off
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);

	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }

      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
    */
/*
    //The train of small spheres
    {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.5;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }
    */


    //The pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = false;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2.5,1, b2Vec2(0.f,-1.9f), 0.85);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.7,4, b2Vec2(-3.8f,1.7f), 0.85);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(6,0.5, b2Vec2(-3.5f,9.f), 0.85);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
      box1->SetGravityScale(0);

       b2PolygonShape shape2;
                shape2.SetAsBox(0.2f, 1.5f);
                b2BodyDef bd2;
                bd2.position.Set(-3.5f,9.f);
                b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = box1;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
      //box1->SetAngularVelocity(0.1);
/*
      //The bar
      bd->position.Set(10,15);
      fd1->density = 34.0;
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);*/
    }
    

    //The revolving horizontal platform
  
    //The heavy sphere on the platform
    
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(28.5f, 42.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    //The main piston
    {
	//The upper piston
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(7.0f, 3.25f);

	b2BodyDef bd;
	bd.type=b2_dynamicBody;
	bd.position.Set(-28.0f, 21.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }

	//The lower revolving body
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(4.0f, 4.0f);

	b2BodyDef bd;
	bd.type = b2_kinematicBody;
	bd.position.Set(-28.0f, 2.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 10.0f);
	
	b4->SetAngularVelocity( 1 );
      }



	//Line joining upper and lower parts of piston
	b2DistanceJointDef jointDef;

	b2Vec2 anchorA;
	b2Vec2 anchorB;

	anchorA.Set(-28.0f,21.0f);
	anchorB.Set(-32.0f,6.0f);

	jointDef.Initialize(b2, b4, anchorA, anchorB);

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
      ballbd.position.Set(-30.0f, 30.0f);
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
      ballbd.position.Set(-17.0f, 20.0f);
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
      ballbd.position.Set(-11.0f, 20.0f);
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
      ballbd.position.Set(-8.0f, 20.0f);
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
      	vertices[0].Set(-35.0f,26.5f);		//done
      	vertices[1].Set(-31.0f,30.0f);		//done
      	vertices[2].Set(-31.9f,31.2f);		//done
      	vertices[3].Set(-34.2f,30.7f);
      	vertices[4].Set(-35.0f,30.0f);
      	vertices[5].Set(-35.7f,27.5f);		
      	
      	poly7.Set(vertices, 6);
      	bd.type=b2_dynamicBody;

      	bp1 = m_world->CreateBody(&bd);
      	b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f;
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

		vertices[0].Set(-35.0f,30.0f);
		vertices[1].Set(-34.2f,30.7f);
		vertices[2].Set(-38.22f,35.8f);
		vertices[3].Set(-39.35f,35.f);

		poly8.Set(vertices,4);
		bd.type = b2_dynamicBody;

		bp2 = m_world->CreateBody(&bd);
		b2FixtureDef *fd = new b2FixtureDef;
      	fd->density = 1.f;
      	fd->shape = new b2PolygonShape;
      	fd->shape = &poly8;
		bp2->CreateFixture(fd);
	    bp2->SetGravityScale(0);
      }

	//Left Valve part 3
      
      b2PolygonShape shape;
      shape.SetAsBox(6.3f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(-34.0f, 38.75f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.0f, 0.0f);
      b2BodyDef bd2;
      bd2.position.Set(-35.0f, 38.75f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = true;

      body->SetTransform( body->GetPosition(), 0.6 );

      m_world->CreateJoint(&jointDef);
      
    

	//Joints of Valve Left     
	b2DistanceJointDef jointDef1;
	b2DistanceJointDef jointDef2;
  	

	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 anchorC;
	b2Vec2 anchorD;
	

	anchorA.Set(-34.2f,30.7f);
	anchorB.Set(-34.2f,30.7f);
	anchorC.Set(-35.0f,30.0f);
	anchorD.Set(-35.0f,30.0f);
  	
	jointDef1.Initialize(bp1, bp2, anchorA, anchorB);
	jointDef2.Initialize(bp1, bp2, anchorC, anchorD);
  	
	jointDef1.collideConnected = true;
	jointDef2.collideConnected = true;
  	
     //b2RevoluteJointDef jd;
     // b2Vec2 anchor;
     // anchor.Set(-28.0f, 35.5f);
      //jd.Initialize(b2, b4,anchor);
    m_world->CreateJoint(&jointDef1);
    m_world->CreateJoint(&jointDef2);
    
      
    }
    
	

//faltu cross in right
{
            //group index -1 here,the wont collide with themslevs
            //change groupindex of other objects
            //this is the new rotating wind mill


            b2PolygonShape shape,shape3;
            shape.SetAsBox(1.6f, 0.2f);
            shape3.SetAsBox(0.2f,1.6f);
            for(int i=0; i<1; i++)
            {
                b2BodyDef bd;
                bd.position.Set(28.5f, 25.7f-2.5f*i);
                if(i%2==0)
                {
                    bd.angle=3.14/2-0.614;
                }
                else
                {
                    bd.angle=3.14/2+0.614;
                }
                bd.type=b2_dynamicBody;
                b2Body* body = m_world->CreateBody(&bd);
                b2FixtureDef *fd = new b2FixtureDef;
                fd->density = 1.f;
                fd->restitution = 0.0f;
                fd->filter.groupIndex = -1;
                fd->shape = new b2PolygonShape;
                fd->shape = &shape;
              
                b2FixtureDef *fd1 = new b2FixtureDef;
                fd1->density = 1.f;
                  fd1->restitution = 1.0f;
                fd1->filter.groupIndex = -1;
                fd1->shape = new b2PolygonShape;
                fd1->shape = &shape3;
                body->CreateFixture(fd);
                body->CreateFixture(fd1);
                //body->SetAngularVelocity(3);



                //useless body
                b2PolygonShape shape2;
                shape2.SetAsBox(0.2f, 1.5f);
                b2BodyDef bd2;
                bd2.position.Set(28.5f, 25.7f-2.5f*i);
                b2Body* body2 = m_world->CreateBody(&bd2);

                //joint is defined here
                b2RevoluteJointDef jointDef;
                jointDef.bodyA = body;
                jointDef.bodyB = body2;
                jointDef.localAnchorA.Set(1,0);
                jointDef.localAnchorB.Set(1,0);
                jointDef.collideConnected = false;
                m_world->CreateJoint(&jointDef);
            }


        }




//both side stand
	//Left stand
    {
      	b2PolygonShape poly;
      	b2BodyDef bd;
      	b2Vec2 vertices[4];
      	vertices[0].Set(-36.0f,7.5f);
      	vertices[1].Set(-35.0f,7.5f);
      	vertices[2].Set(-35.0f,26.5f);
      	vertices[3].Set(-36.0f,28.0f);
      	poly.Set(vertices, 4);
      	bd.type=b2_staticBody;

      	b2Body* body = m_world->CreateBody(&bd);
      	 b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &poly;
      body->CreateFixture(fd);
      	

    }
	//Right stand
    {
      	b2PolygonShape poly1;
      	b2BodyDef bd1;
      	b2Vec2 vertices[4];
      	vertices[0].Set(-20.0f,7.5f);
      	vertices[1].Set(-21.0f,7.5f);
      	vertices[2].Set(-21.0f,26.5f);
      	vertices[3].Set(-20.0f,28.5f);
      	poly1.Set(vertices, 4);
      	bd1.type=b2_staticBody;

      	b2Body* body = m_world->CreateBody(&bd1);
      	 b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.f;
      fd1->shape = new b2PolygonShape;
      fd1->shape = &poly1;
      body->CreateFixture(fd1);
      	
      	
    }
 

	//Rotating triangle
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_kinematicBody;
      bd->position.Set(-29.0f,40.1547f);
      bd->fixedRotation = true;
       //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10000000.0;
      fd1->friction = 1.0;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;

      b2PolygonShape bs1;
      bs1.SetAsBox(2.5,0.2, b2Vec2(0.f,-1.75f), 0);



      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10000000.0;
      fd2->friction = 0.0;
      fd2->restitution = 1.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2.5, b2Vec2(1.0f,0.f), 0.523598775f);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10000000.0;
      fd3->friction = 0.0;
      fd3->restitution = 1.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2.5, b2Vec2(-1.0f,0.f), -0.523598775f);
      fd3->shape = &bs3;


      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
      box1->SetGravityScale(0);
      box1->SetAngularVelocity(-0.5);

    }
/*
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = false;
       //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(4,0.7, b2Vec2(0.f,-1.9f), 0.2);



      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(4,0.7, b2Vec2(7.0f,-2.f), -0.2f);
      fd2->shape = &bs2;
      //b2FixtureDef *fd3 = new b2FixtureDef;
      //fd3->density = 10.0;
      //fd3->friction = 0.5;
      //fd3->restitution = 0.f;
      //fd3->shape = new b2PolygonShape;
      //b2PolygonShape bs3;
      //bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), -0.523598775f);
      //fd3->shape = &bs3;


      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      //box1->CreateFixture(fd3);
      box1->SetGravityScale(0);
      box1->SetAngularVelocity(0);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);

    }
*/
    /*
    {
      b2Body* b2;
      {
  b2PolygonShape shape;
  shape.SetAsBox(4,0.7, b2Vec2(0.f,-1.9f), 0.2);

  b2BodyDef bd;
  bd.type=b2_dynamicBody;
  b2 = m_world->CreateBody(&bd);
  b2->CreateFixture(&shape, 10.0f);
  b2->SetGravityScale(0);
      }

      b2Body* b4;
      {
  b2PolygonShape shape;
  shape.SetAsBox(4,0.7, b2Vec2(7.0f,-2.f), -0.2f);

  b2BodyDef bd;
  bd.type = b2_dynamicBody;
  b4 = m_world->CreateBody(&bd);
  b4->CreateFixture(&shape, 10.0f);
  b4->SetGravityScale(0);
  //b4->SetAngularVelocity( 1 );
      }




  //b2DistanceJointDef jointDef;

  //b2Vec2 anchorA;
  //b2Vec2 anchorB;

  //anchorA.Set(-28.0f,21.0f);
  //anchorB.Set(-32.0f,6.0f);

  //jointDef.Initialize(b2, b4, anchorA, anchorB);

  //jointDef.collideConnected = true;


      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(7.0f,-2.f);
      jd.Initialize(b2, b4,anchor);
      m_world->CreateJoint(&jd);
    }
*/
    {
        b2PolygonShape poly17;
        b2BodyDef bd1;
        b2Vec2 vertices[8];
        vertices[0].Set(0.0f,0.0f);
        vertices[1].Set(10.0f,0.0f);
        vertices[2].Set(10.0f,5.0f);
        vertices[3].Set(6.0f,5.0f);
        vertices[4].Set(6.0f,10.0f);
        vertices[5].Set(4.0f,10.0f);
        vertices[6].Set(4.0f,5.0f);
        vertices[7].Set(0.0f,5.0f);
        poly17.Set(vertices, 8);
        bd1.type=b2_staticBody;

        b2Body* body = m_world->CreateBody(&bd1);
         b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.f;
      fd1->shape = new b2PolygonShape;
      fd1->shape = &poly17;
      body->CreateFixture(fd1);
        
        }


    //just uppar stand
    {
    	b2PolygonShape poly1;
      	b2BodyDef bd1;
      	b2Vec2 vertices[4];
      	vertices[0].Set(-31.0f,30.0f);
      	vertices[1].Set(-25.0f,30.0f);
      	vertices[2].Set(-23.0f,34.0f);
      	vertices[3].Set(-33.0f,34.0f);
      	poly1.Set(vertices, 4);
      	bd1.type=b2_staticBody;

      	b2Body* body = m_world->CreateBody(&bd1);
      	 b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.f;
      fd1->shape = new b2PolygonShape;
      fd1->shape = &poly1;
      body->CreateFixture(fd1);
      	
    }

  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
